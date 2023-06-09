/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "lvgl.h"
#include "hal/hal_gpio.h"

#define BROADCAST_ASSISTANT_ADD_SOURCE_OPCODE          0x02
#define BROADCAST_ASSISTANT_MODIFY_SOURCE_OPCODE       0x03
#define BROADCAST_ASSISTANT_REMOVE_SOURCE_OPCODE       0x05

#define PA_SYNC_NOT_SYNCHRONIZE                        0x00
#define PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE       0x01

#define COLOR_DEVICE                        0x35281D
#define COLOR_DEVICE_SPECIAL_STATE          0x7D70BA

#define LTV_STRUCT_STREAMING_AUDIO_CONTEXT  0x02
#define LTV_STRUCT_PROGRAM_INFO             0x03
#define LTV_STRUCT_LANGUAGE                 0x04

#define MAX_DEVICES                         MYNEWT_VAL(MAX_DEVICES)
#define MAX_SUBGROUPS                       MYNEWT_VAL(MAX_SUBGROUPS)
#define MAX_BIS                             MYNEWT_VAL(MAX_BIS)

#define LIST_ID_NONE                        0xff

#define DEVICE_ROLE_BROADCAST_SOURCE        0
#define DEVICE_ROLE_BROADCAST_DELEGATOR     1

static const ble_uuid16_t uuid_broadcast_audio_scan_svc = BLE_UUID16_INIT(0x184F);
static const ble_uuid16_t uuid_broadcast_audio_scan_control_point_chr = BLE_UUID16_INIT(0x2BC7);
static const ble_uuid16_t uuid_broadcast_receive_state_chr = BLE_UUID16_INIT(0x2BC8);

struct display_gui {
    lv_style_t style;
    lv_obj_t *devices_list;
    lv_obj_t *filters_list;
    lv_obj_t *subgroups_list;
    lv_obj_t *scan_btn;
    lv_obj_t *filter_btn;
    lv_obj_t *return_btn;
    lv_obj_t *remove_src_btn;
};

struct broadcast_device_data {
    int8_t last_rssi;
    ble_addr_t addr;
    lv_obj_t *list_entry;
    uint8_t device_role;
    uint8_t sid;
    uint32_t broadcast_id;
};

struct filter_settings {
    int8_t rssi;
    uint8_t device_role;
};

struct paired_delegator {
    uint8_t list_id;
    uint16_t conn_handle;
    uint16_t broadcast_audio_scan_service_start_h;
    uint16_t broadcast_audio_scan_service_end_h;
    uint16_t scan_control_point_h;
    uint16_t broadcast_receive_state_h;
};

struct bis {
    uint8_t idx;
    lv_obj_t *list_entry;
};

struct base_subgroup {
    uint8_t metadata_len;
    uint8_t metadata[255];
    uint8_t num_bis;
    struct bis bises[MAX_BIS];
};

struct synced_source {
    uint8_t list_id;
    uint8_t server_id;
    uint16_t sync_handle;
    uint16_t pa_interval;
    uint8_t num_subgroups;
    struct base_subgroup subgroups[MAX_SUBGROUPS];
    uint8_t is_added;
    uint8_t is_server_synced;
};

static uint8_t g_own_addr_type;

static uint8_t scanning = 0;
static uint8_t create_new_subgroups_list_flag = 1;
static uint8_t sort_suspended = 0;
static struct display_gui gui;
static struct broadcast_device_data devices[MAX_DEVICES];

static struct filter_settings filter_settings = {
        .rssi = -75,
        .device_role = DEVICE_ROLE_BROADCAST_SOURCE,
};

static struct synced_source synced_source = {
        .list_id = LIST_ID_NONE,
};

static struct paired_delegator paired_delegators[2] = {
        [0].list_id = LIST_ID_NONE,
        [1].list_id = LIST_ID_NONE,
        [0].conn_handle = BLE_HS_CONN_HANDLE_NONE,
        [1].conn_handle = BLE_HS_CONN_HANDLE_NONE,
};

static void start_scan(void);
static void stop_scan(void);
static void server_add_new_source(struct paired_delegator *delegator, uint8_t source_list_id, uint32_t bis_sync);
static void server_modify_source(struct paired_delegator *delegator, uint32_t bis_sync, uint8_t pa_sync);

static void
switch_to_subgroups_list(void)
{
    lv_obj_move_foreground(gui.subgroups_list);
    lv_obj_clear_flag(gui.return_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(gui.remove_src_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(gui.scan_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(gui.filter_btn, LV_OBJ_FLAG_HIDDEN);
}

static void
switch_to_devices_list(void)
{
    lv_obj_move_foreground(gui.devices_list);
    lv_obj_clear_flag(gui.scan_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(gui.filter_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(gui.return_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(gui.remove_src_btn, LV_OBJ_FLAG_HIDDEN);
}

static int
rssi_compare(const void *p1, const void *p2)
{
    const struct broadcast_device_data *d1 = p1;
    const struct broadcast_device_data *d2 = p2;

    return (d2->last_rssi - d1->last_rssi);
}

static void
sort_devices(void)
{
    uint8_t i;
    uint8_t devices_number = lv_obj_get_child_cnt(gui.devices_list);

    qsort(devices, devices_number, sizeof(*devices), rssi_compare);

    for (i = 0; i < devices_number; i++) {
        lv_obj_move_to_index(devices[i].list_entry, i);
    }
}

static int
get_device_id_by_addr(ble_addr_t *addr)
{
    uint8_t id;
    uint8_t n = lv_obj_get_child_cnt(gui.devices_list);
    assert (n <= MAX_DEVICES);

    for (id = 0; id < n; id++) {
        if (!ble_addr_cmp(addr, &devices[id].addr)) {
            return id;
        }
    }
    return -1;
}

static void
fill_device_labels(struct ble_gap_ext_disc_desc *desc, uint8_t id)
{
    lv_obj_t *entry = devices[id].list_entry;
    lv_obj_t *label;
    struct ble_hs_adv_fields fields;
    char name_buf[255];

    ble_hs_adv_parse_fields(&fields, desc->data, desc->length_data);

    label = lv_obj_get_child(entry, 0);
    lv_label_set_text_fmt(label, "%02x:%02x:%02x:%02x:%02x:%02x (%u)", desc->addr.val[5],
                          desc->addr.val[4], desc->addr.val[3], desc->addr.val[2],
                          desc->addr.val[1], desc->addr.val[0], desc->addr.type);

    label = lv_obj_get_child(entry, 1);
    lv_label_set_text_fmt(label, "%ddB", desc->rssi);

    label = lv_obj_get_child(entry, 2);
    lv_label_set_text_fmt(label, "%s", desc->prim_phy == BLE_HCI_LE_PHY_1M ? "1M" : "Coded");
    switch (desc->sec_phy) {
    case BLE_HCI_LE_PHY_1M:
        lv_label_ins_text(label, LV_LABEL_POS_LAST, "/1M");
        break;
    case BLE_HCI_LE_PHY_2M:
        lv_label_ins_text(label, LV_LABEL_POS_LAST, "/2M");
        break;
    case BLE_HCI_LE_PHY_CODED:
        lv_label_ins_text(label, LV_LABEL_POS_LAST, "/Coded");
        break;
    default:
        break;
    }

    if (fields.name != NULL) {
        label = lv_obj_get_child(entry, 3);
        memcpy(name_buf, fields.name, fields.name_len);
        name_buf[fields.name_len] = '\0';
        lv_label_set_text_fmt(label, "%s", name_buf);
    }

    if (devices[id].broadcast_id != UINT32_MAX) {
        label = lv_obj_get_child(entry, 4);
        lv_label_set_text_fmt(label, "ID:0x%lx", devices[id].broadcast_id);
    }
}

static void
create_device_labels(uint8_t id)
{
    lv_obj_t *entry;
    lv_obj_t *label;

    entry = devices[id].list_entry;

    /* Label for device address */
    label = lv_label_create(entry);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);
    /* Label for RSSI */
    label = lv_label_create(entry);
    lv_obj_align(label, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    /* Label for PHY */
    label = lv_label_create(entry);
    lv_obj_align_to(label, lv_obj_get_child(entry, 1), LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    /* Label for Name */
    label = lv_label_create(entry);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_label_set_text(label, "");
    /* Label for Broadcast ID */
    label = lv_label_create(entry);
    lv_obj_align_to(label, lv_obj_get_child(entry, 2), LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_label_set_text(label, "");
}

static void
subgroups_list_init(void)
{
    gui.subgroups_list = lv_obj_create(lv_scr_act());
    lv_obj_set_size(gui.subgroups_list, LV_PCT(100), LV_PCT(80));
    lv_obj_align(gui.subgroups_list, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_set_flex_flow(gui.subgroups_list, LV_FLEX_FLOW_COLUMN);
}

static void
parse_synced_source_base(const uint8_t *base)
{
    int i;
    int j;

    /* First parameter we need is num_subgroups, which is 7th byte of advertising packet */
    base += 7;
    synced_source.num_subgroups = *(base++);

    /* fixme */
    assert(synced_source.num_subgroups <= MAX_SUBGROUPS);

    for(i = 0; i < synced_source.num_subgroups; i++) {
        synced_source.subgroups[i].num_bis = *base;

        assert(synced_source.subgroups[i].num_bis <= MAX_BIS);

        /* Skip parameters up to metadata_length[i] */
        base += 7 + base[6];

        synced_source.subgroups[i].metadata_len = *(base++);
        memcpy(synced_source.subgroups[i].metadata, base, synced_source.subgroups[i].metadata_len);

        base += synced_source.subgroups[i].metadata_len;

        for (j = 0; j < synced_source.subgroups[i].num_bis; j++) {
            synced_source.subgroups[i].bises[j].idx = *base;
            /* Skip Codec_Specific_Configuration */
            base += base[1] + 2;
        }
    }
}

static void
server_unsync_source(struct paired_delegator *delegator)
{
    server_modify_source(delegator, 0, PA_SYNC_NOT_SYNCHRONIZE);
}

static void
bis_click_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_obj_t *label = lv_obj_get_child(obj, 1);
    char *bis_idx_ascii = lv_label_get_text(label);
    uint32_t bis_idx = atoi(bis_idx_ascii);

    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            if (synced_source.is_added) {
                server_modify_source(&paired_delegators[0], bis_idx, PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE);
            } else {
                server_add_new_source(&paired_delegators[0], synced_source.list_id, bis_idx);
            }
        }
        if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            if (synced_source.is_added) {
                server_modify_source(&paired_delegators[1], bis_idx, PA_SYNC_SYNCHRONIZE_TO_PA_PAST_AVAILABLE);
            } else {
                server_add_new_source(&paired_delegators[1], synced_source.list_id, bis_idx);
            }
        }
    } else if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
        if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            server_unsync_source(&paired_delegators[0]);
        }
        if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            server_unsync_source(&paired_delegators[1]);
        }
    }
}

static void
create_new_subgroups_list(void)
{
    int i;
    int j;
    char program_info_buf[50];
    lv_obj_t *obj;
    lv_obj_t *label;
    uint8_t metadata_len;
    uint8_t ltv_struct_type;
    uint8_t ltv_struct_len;
    uint8_t buff_ptr;

    buff_ptr = 0;
    lv_obj_del(gui.subgroups_list);
    subgroups_list_init();

    for(i = 0; i < synced_source.num_subgroups; i++) {
        obj = lv_obj_create(gui.subgroups_list);
        lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);
        label = lv_label_create(obj);
        lv_label_set_text_fmt(label, "Subgroup %d", i);

        metadata_len = synced_source.subgroups[i].metadata_len;

        while (metadata_len != 0) {

            assert(metadata_len >= 0);

            ltv_struct_len = synced_source.subgroups[i].metadata[buff_ptr++];
            ltv_struct_type = synced_source.subgroups[i].metadata[buff_ptr++];

            switch (ltv_struct_type) {
            case LTV_STRUCT_STREAMING_AUDIO_CONTEXT:
                label = lv_label_create(obj);
                lv_label_set_text_fmt(label, "Audio context: %d", get_le16(&synced_source.subgroups[i].metadata[buff_ptr]));
                break;
            case LTV_STRUCT_PROGRAM_INFO:
                label = lv_label_create(obj);
                memcpy(&program_info_buf, &synced_source.subgroups[i].metadata[buff_ptr], ltv_struct_len - 1);
                program_info_buf[ltv_struct_len - 1] = '\0';
                lv_label_set_text_fmt(label, "Program Info:\n %s", program_info_buf);
                break;
            case LTV_STRUCT_LANGUAGE: //todo
                break;
            }
            metadata_len -= ltv_struct_len + 1;
            buff_ptr += ltv_struct_len - 1;
        }
        label = lv_label_create(obj);
        lv_label_set_text_fmt(label, "Num BIS: %d", synced_source.subgroups[i].num_bis);

        for (j = 0; j < synced_source.subgroups[i].num_bis; j++) {
            obj = lv_btn_create(gui.subgroups_list);
            lv_obj_set_size(obj, LV_PCT(100), LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);
            label = lv_label_create(obj);
            lv_label_set_text(label, "BIS\n");
            label = lv_label_create(obj);
            lv_label_set_text_fmt(label, "%d", synced_source.subgroups[i].bises[j].idx);
            lv_obj_add_event_cb(obj, bis_click_event_cb, LV_EVENT_ALL, NULL);
            lv_obj_set_style_bg_color(obj, lv_color_hex(COLOR_DEVICE), 0);
            synced_source.subgroups[i].bises[j].list_entry = obj;
        }
    }
    create_new_subgroups_list_flag = 0;
    switch_to_subgroups_list();
}

static int
broadcast_assistant_transfer_syncinfo(uint8_t source_id, uint8_t paired_delegator_id)
{
    return ble_gap_periodic_adv_sync_transfer(synced_source.sync_handle,
                                              paired_delegators[paired_delegator_id].conn_handle, (uint16_t)source_id);
}

static uint8_t
bis_get_associated_subgroup_idx(uint8_t bis_idx)
{
    int i;
    int j;

    for(i = 0; i < synced_source.num_subgroups; i++) {
        for(j = 0; j < synced_source.subgroups[i].num_bis; j++) {
            if (bis_idx == synced_source.subgroups[i].bises[j].idx) {
                return i;
            }
        }
    }
    return 0xff;
}

static void
server_modify_source(struct paired_delegator *delegator, uint32_t bis_sync, uint8_t pa_sync)
{
    uint8_t data_to_send[255];
    uint16_t data_len;
    uint8_t subgroup_idx;

    subgroup_idx = bis_get_associated_subgroup_idx((uint8_t) bis_sync);

    if (subgroup_idx == 0xff && pa_sync != PA_SYNC_NOT_SYNCHRONIZE) {
        return;
    }

    data_to_send[0] = BROADCAST_ASSISTANT_MODIFY_SOURCE_OPCODE;

    /* Source_ID */
    data_to_send[1] = synced_source.server_id;

    /* PA_sync */
    data_to_send[2] = pa_sync;

    /* PA_Interval */
    memcpy(&data_to_send[3], &synced_source.pa_interval, 2);

    /* num_subgroups */
    data_to_send[5] = synced_source.num_subgroups;

    /* BIS_Sync */
    memcpy(&data_to_send[6], &bis_sync, 4);

    /* metadata */
    data_to_send[10] = synced_source.subgroups[subgroup_idx].metadata_len;
    memcpy(&data_to_send[11], synced_source.subgroups[subgroup_idx].metadata, synced_source.subgroups[subgroup_idx].metadata_len);

    data_len = 11 + synced_source.subgroups[subgroup_idx].metadata_len;

    ble_gattc_write_no_rsp_flat(delegator->conn_handle, delegator->scan_control_point_h, data_to_send, data_len);
}

static void
server_add_new_source(struct paired_delegator *delegator, uint8_t source_list_id, uint32_t bis_sync)
{
    uint8_t data_to_send[255];
    uint16_t data_len;
    uint8_t subgroup_idx;

    subgroup_idx = bis_get_associated_subgroup_idx((uint8_t) bis_sync);

    if (subgroup_idx == 0xff) {
        return;
    }

    data_to_send[0] = BROADCAST_ASSISTANT_ADD_SOURCE_OPCODE;

    /* Advertiser_Address_Type */
    data_to_send[1] = devices[source_list_id].addr.type;

    /* Advertiser_Address */
    memcpy(&data_to_send[2], &devices[source_list_id].addr.val, 6);

    /* Advertising_SID */
    data_to_send[8] = devices[source_list_id].sid;

    /* Broadcast_ID */
    memcpy(&data_to_send[9], &devices[source_list_id].broadcast_id, 3);

    /* PA_sync */
    data_to_send[12] = 0x01;

    /* PA_Interval */
    memcpy(&data_to_send[13], &synced_source.pa_interval, 2);

    /* num_subgroups */
    data_to_send[15] = synced_source.num_subgroups;

    /* BIS_Sync */
    put_le32(&data_to_send[16], bis_sync);

    /* metadata */
    data_to_send[20] = synced_source.subgroups[subgroup_idx].metadata_len;
    memcpy(&data_to_send[21], synced_source.subgroups[subgroup_idx].metadata, synced_source.subgroups[subgroup_idx].metadata_len);

    data_len = 21 + synced_source.subgroups[subgroup_idx].metadata_len;

    ble_gattc_write_flat(delegator->conn_handle, delegator->scan_control_point_h, data_to_send, data_len, NULL, NULL);
}

static int
periodic_adv_sync_cb_func(struct ble_gap_event *event, void *arg)
{
    uint8_t id = *((uint8_t *) arg);

    switch (event->type) {
    case BLE_GAP_EVENT_PERIODIC_SYNC:
        printf("periodic sync status: %d\n", event->periodic_sync.status);

        if (!event->periodic_sync.status) {
            synced_source.list_id = id;
            synced_source.sync_handle = event->periodic_sync.sync_handle;
            synced_source.pa_interval = event->periodic_sync.per_adv_ival;
            stop_scan();
        } else {
            synced_source.list_id = LIST_ID_NONE;
        }
        break;
    case BLE_GAP_EVENT_PERIODIC_REPORT:
        if (create_new_subgroups_list_flag) {
            parse_synced_source_base(event->periodic_report.data);
            create_new_subgroups_list();
        }
        break;
    case BLE_GAP_EVENT_PERIODIC_SYNC_LOST:
        printf("Periodic sync lost\n");
        synced_source.list_id = LIST_ID_NONE;
        create_new_subgroups_list_flag = 1;
        break;
    case BLE_GAP_EVENT_BIGINFO_REPORT:
        //printf("Got BIGInfo report!\n");
        break;
    }

    return 0;
}

static int
disc_dsc_cb_func(uint16_t conn_handle, const struct ble_gatt_error *error,
                 uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    struct paired_delegator *delegator = arg;

    uint8_t value[2];
    value[0] = 1;
    value[1] = 0;

    if ((chr_val_handle == delegator->broadcast_receive_state_h) &&
        (dsc->uuid.u16.value == BLE_GATT_DSC_CLT_CFG_UUID16)) {
        ble_gattc_write_flat(conn_handle, dsc->handle,
                             value, sizeof value, NULL, NULL);
    }
    return 0;
}

static int
disc_chr_cb_func(uint16_t conn_handle, const struct ble_gatt_error *error,
                 const struct ble_gatt_chr *chr, void *arg)
{
    struct paired_delegator *delegator = arg;
    assert(conn_handle == delegator->conn_handle);

    if (error->status == BLE_HS_EDONE) {
        ble_gattc_disc_all_dscs(conn_handle, delegator->broadcast_receive_state_h,
                                delegator->broadcast_receive_state_h + 1, disc_dsc_cb_func, delegator);
    } else if (!ble_uuid_cmp(&chr->uuid.u, &uuid_broadcast_audio_scan_control_point_chr.u)) {
        delegator->scan_control_point_h = chr->val_handle;
        printf("Broadcast Audio Scan Control Point Characteristic found [%04x]\n", chr->val_handle);
    } else if (!ble_uuid_cmp(&chr->uuid.u, &uuid_broadcast_receive_state_chr.u)) {
        delegator->broadcast_receive_state_h = chr->val_handle;
        printf("Broadcast Receive State Characteristic found [%04x]\n", chr->val_handle);
    }
    return 0;
}

static int
disc_svc_cb_func(uint16_t conn_handle, const struct ble_gatt_error *error,
                 const struct ble_gatt_svc *service, void *arg)
{
    struct paired_delegator *delegator = arg;
    assert(conn_handle == delegator->conn_handle);

    int rc = 0;

    if (error->status == BLE_HS_EDONE) {
        printf("Found Broadcast Audio Scan Service [%04x %04x]\n", service->start_handle,
               service->end_handle);

        rc = ble_gattc_disc_all_chrs(delegator->conn_handle, delegator->broadcast_audio_scan_service_start_h + 1,
                                     delegator->broadcast_audio_scan_service_end_h, disc_chr_cb_func, delegator);
    } else if (!ble_uuid_cmp(&service->uuid.u, &uuid_broadcast_audio_scan_svc.u)) {
        delegator->broadcast_audio_scan_service_start_h = service->start_handle;
        delegator->broadcast_audio_scan_service_end_h = service->end_handle;
    }

    return rc;
}

static int
exchange_mtu_cb_func(uint16_t conn_handle, const struct ble_gatt_error *error,
                     uint16_t mtu, void *arg)
{
    if (error->status) {
        printf("MTU exchange error: %d", error->status);
    } else {
        printf("MTU exchange success\n");
    }
    return 0;
}

static void
tag_synced_bis(uint32_t bis_sync_state)
{
    int i;
    int j;
    for (i = 0; i < synced_source.num_subgroups; i++) {
        for(j = 0; j < synced_source.subgroups[i].num_bis; j++) {
            if (synced_source.subgroups[i].bises[j].idx == (uint8_t) bis_sync_state) {
                lv_obj_set_style_bg_color(synced_source.subgroups[i].bises[j].list_entry,
                                          lv_color_hex(COLOR_DEVICE_SPECIAL_STATE), 0);
            }
        }
    }
}

static void
untag_bises(void)
{
    int i;
    int j;
    for (i = 0; i < synced_source.num_subgroups; i++) {
        for(j = 0; j < synced_source.subgroups[i].num_bis; j++) {
            lv_obj_set_style_bg_color(synced_source.subgroups[i].bises[j].list_entry,
                                      lv_color_hex(COLOR_DEVICE), 0);

        }
    }
}

static int
connect_cb_func(struct ble_gap_event *event, void *arg)
{
    int rc;
    uint8_t id;
    uint32_t bis_sync_state;
    lv_obj_t *entry;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        rc = ble_gap_security_initiate(event->connect.conn_handle);
        printf("Pairing initiate rc = %d\n", rc);
        break;
    case BLE_GAP_EVENT_PARING_COMPLETE:
        printf("Pairing complete status: %d\n", event->pairing_complete.status);
        if (!event->pairing_complete.status) {
            id = *((uint8_t *) arg);
            if (paired_delegators[0].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                paired_delegators[0].conn_handle = event->pairing_complete.conn_handle;
                paired_delegators[0].list_id = id;
                ble_gattc_disc_svc_by_uuid(paired_delegators[0].conn_handle,
                                           &uuid_broadcast_audio_scan_svc.u, disc_svc_cb_func,
                                           &paired_delegators[0]);
            } else if (paired_delegators[1].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                paired_delegators[1].conn_handle = event->pairing_complete.conn_handle;
                paired_delegators[1].list_id = id;
                ble_gattc_disc_svc_by_uuid(paired_delegators[1].conn_handle,
                                           &uuid_broadcast_audio_scan_svc.u, disc_svc_cb_func,
                                           &paired_delegators[1]);
            } else {
                return 1;
            }
            ble_gattc_exchange_mtu(event->pairing_complete.conn_handle, exchange_mtu_cb_func, NULL);
            entry = devices[id].list_entry;
            lv_obj_set_style_bg_color(entry, lv_color_hex(COLOR_DEVICE_SPECIAL_STATE), 0);
        }
        break;
    case BLE_GAP_EVENT_NOTIFY_RX:
        printf("Notification received\n");
        if (event->notify_rx.attr_handle == paired_delegators->broadcast_receive_state_h) {
            /* This means that source was removed from server */
            if (event->notify_rx.om->om_len == 0) {
                synced_source.is_added = 0;
            }
            switch (*(event->notify_rx.om->om_data + 12)) {
            case 0x00:
                if (synced_source.is_server_synced) {
                    untag_bises();
                    synced_source.is_server_synced = 0;
                }
                break;
            case 0x01:
                synced_source.server_id = *event->notify_rx.om->om_data;
                synced_source.is_added = 1;
                if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    broadcast_assistant_transfer_syncinfo(synced_source.server_id, 0);
                }
                if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    broadcast_assistant_transfer_syncinfo(synced_source.server_id, 1);
                }
                break;
            case 0x02:
                bis_sync_state = get_le32(event->notify_rx.om->om_data + 15);
                if (bis_sync_state == 0 || bis_sync_state == 0xffffffff) {
                    return 1;
                }
                synced_source.is_server_synced = 1;
                tag_synced_bis(bis_sync_state);
            }
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        printf("Disconnected from delegator\n");
        if (event->disconnect.conn.conn_handle == paired_delegators[0].conn_handle) {
            id = paired_delegators[0].list_id;
            paired_delegators[0].list_id = LIST_ID_NONE;
            paired_delegators[0].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        } else if (event->disconnect.conn.conn_handle == paired_delegators[1].conn_handle) {
            id = paired_delegators[1].list_id;
            paired_delegators[1].list_id = LIST_ID_NONE;
            paired_delegators[1].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        } else {
            return 1;
        }
        entry = devices[id].list_entry;
        lv_obj_set_style_bg_color(entry, lv_color_hex(COLOR_DEVICE), 0);
        break;
    }
    return 0;
}

static void
entry_click_event_cb(lv_event_t *e)
{
    static uint8_t id;
    int rc;
    struct ble_gap_periodic_sync_params periodic_sync_params;

    periodic_sync_params.skip = 0;
    periodic_sync_params.sync_timeout = 0x4000;
    periodic_sync_params.reports_disabled = 0;

    lv_obj_t *entry = lv_event_get_target(e);
    id = lv_obj_get_index(entry);

    if (filter_settings.device_role == DEVICE_ROLE_BROADCAST_SOURCE) {
        sort_suspended = 1;
        if (!ble_gap_disc_active()) {
            start_scan();
        }
        rc = ble_gap_periodic_adv_sync_create(&devices[id].addr, devices[id].sid,
                                              &periodic_sync_params,
                                              periodic_adv_sync_cb_func, &id);
        printf("Periodic advertising sync create rc = %d\n", rc);
    } else {
        if (paired_delegators[0].list_id == id) {
            ble_gap_terminate(paired_delegators[0].conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        } else if (paired_delegators[1].list_id == id) {
            ble_gap_terminate(paired_delegators[1].conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        } else {
            rc = ble_gap_connect(g_own_addr_type, &devices[id].addr, 10000,
                                 NULL, connect_cb_func, &id);
            printf("Connection rc = %d\n", rc);
        }
    }
}

static void
add_new_device(struct ble_gap_ext_disc_desc *desc, uint32_t broadcast_id)
{
    uint8_t id;

    lv_obj_t *entry = lv_btn_create(gui.devices_list);
    lv_obj_add_event_cb(entry, entry_click_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_size(entry, LV_PCT(100), 40);
    lv_obj_add_style(entry, &gui.style, 0);

    id = lv_obj_get_child_cnt(gui.devices_list) - 1;
    devices[id].device_role = filter_settings.device_role;
    devices[id].list_entry = entry;
    devices[id].last_rssi = desc->rssi;
    devices[id].sid = desc->sid;
    devices[id].broadcast_id = broadcast_id;
    memcpy(&devices[id].addr, &desc->addr, 7);

    create_device_labels(id);
    fill_device_labels(desc, id);
}

static void
update_device(struct ble_gap_ext_disc_desc *desc, uint8_t id)
{
    uint8_t rssi_change;
    fill_device_labels(desc, id);

    /* Sort the list if and update RSSI if it changed significantly */
    if (!sort_suspended) {
        rssi_change = abs(devices[id].last_rssi - desc->rssi);
        if (rssi_change > 5) {
            devices[id].last_rssi = desc->rssi;
            sort_devices();
        }
    }
}

static void
devices_list_init(void)
{
    gui.devices_list = lv_obj_create(lv_scr_act());
    lv_obj_set_size(gui.devices_list, LV_PCT(100), LV_PCT(80));
    lv_obj_align(gui.devices_list, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_set_flex_flow(gui.devices_list, LV_FLEX_FLOW_COLUMN);
}

static int
parse_broadcast_audio_announcement(const uint8_t *data, uint8_t src_len, uint32_t *broadcast_id_dst)
{
    uint8_t ad_len;
    uint8_t ad_type;
    uint16_t svc_uuid_val;

    while (src_len > 0) {
        ad_len = data[0];
        ad_type = data[1];
        if (ad_type == 0x16) {
            svc_uuid_val = get_le16(&data[2]);
            if (svc_uuid_val == 0x1852) {
                *broadcast_id_dst = get_le24(&data[4]);
                return 1;
            }
        }
        data += ad_len + 1;
        src_len -= ad_len + 1;
    }
    return 0;
}

static int
is_device_scan_delegator(const uint8_t *data, uint8_t src_len)
{
    uint8_t ad_len;
    uint8_t ad_type;
    uint16_t svc_uuid_val;

    while (src_len > 0) {
        ad_len = data[0];
        ad_type = data[1];
        if (ad_type == 0x16) {
            svc_uuid_val = get_le16(&data[2]);
            /* FIXME: this should be uuid_broadcast_audio_scan_svc.u but samsung/sony does not adv this svc data */
            if (svc_uuid_val == 0x184E) {
                return 1;
            }
        }
        data += ad_len + 1;
        src_len -= ad_len + 1;
    }
    return 0;
}

static void
handle_delegator_disc_report(struct ble_gap_ext_disc_desc *desc)
{
    int id;

    if (!is_device_scan_delegator(desc->data, desc->length_data) || desc->rssi < filter_settings.rssi) {
        return;
    }

    if (lv_obj_get_child_cnt(gui.devices_list) >= MAX_DEVICES) {
        lv_obj_del(gui.devices_list);
        devices_list_init();
    }

    id = get_device_id_by_addr(&desc->addr);

    if (id < 0) {
        add_new_device(desc, UINT32_MAX);
        printf("New device\n");
        if (!sort_suspended) {
            sort_devices();
        }
    } else {
        update_device(desc, id);
    }
}

static void
handle_source_disc_report(struct ble_gap_ext_disc_desc *desc)
{
    int device_id;
    uint32_t ad_broadcast_id;

    /* This function returning 0 means, that discovered device is not Broadcast Source */
    if (!parse_broadcast_audio_announcement(desc->data, desc->length_data,
                                            &ad_broadcast_id)) {
        return;
    }

    if (lv_obj_get_child_cnt(gui.devices_list) >= MAX_DEVICES) {
        lv_obj_del(gui.devices_list);
        devices_list_init();
    }

    device_id = get_device_id_by_addr(&desc->addr);

    if (device_id < 0) {
        add_new_device(desc, ad_broadcast_id);
        if (!sort_suspended) {
            sort_devices();
        }
    } else {
        update_device(desc, device_id);
    }
}

static int
scan_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_EXT_DISC:
        if (filter_settings.device_role == DEVICE_ROLE_BROADCAST_SOURCE) {
            handle_source_disc_report(&event->ext_disc);
        } else if (filter_settings.device_role == DEVICE_ROLE_BROADCAST_DELEGATOR) {
            handle_delegator_disc_report(&event->ext_disc);
        } else {
            assert(0);
        }
        break;
    }
    return 0;
}

static void
stop_scan(void)
{
    int rc;
    const uint8_t remote_scan_stopped = 0x00;

    rc = ble_gap_disc_cancel();
    if (!rc) {
        scanning = 0;
        lv_label_set_text(lv_obj_get_child(gui.scan_btn, 0), "Start\nscan");

        if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gattc_write_flat(paired_delegators[0].conn_handle, paired_delegators[0].scan_control_point_h,
                                 &remote_scan_stopped, sizeof(remote_scan_stopped), NULL, NULL);
        }
        if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gattc_write_flat(paired_delegators[1].conn_handle, paired_delegators[1].scan_control_point_h,
                                 &remote_scan_stopped, sizeof(remote_scan_stopped), NULL, NULL);
        }
    }
}

static void
start_scan(void)
{
    int rc;
    struct ble_gap_ext_disc_params coded;
    struct ble_gap_ext_disc_params uncoded;
    const uint8_t remote_scan_started = 0x01;

    coded.passive = 0;
    coded.itvl = BLE_GAP_SCAN_ITVL_MS(500);
    coded.window = BLE_GAP_SCAN_WIN_MS(500);
    uncoded.passive = 0;
    uncoded.itvl = BLE_GAP_SCAN_ITVL_MS(500);
    uncoded.window = BLE_GAP_SCAN_WIN_MS(500);

    rc = ble_gap_ext_disc(BLE_OWN_ADDR_RANDOM, 0, 0, 0, 0,
                          0, &uncoded, &coded, scan_event, NULL);
    if (!rc) {
        scanning = 1;
        lv_label_set_text(lv_obj_get_child(gui.scan_btn, 0), "Stop\nscan");

        if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gattc_write_flat(paired_delegators[0].conn_handle, paired_delegators[0].scan_control_point_h,
                                 &remote_scan_started, sizeof(remote_scan_started), NULL, NULL);
        }
        if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gattc_write_flat(paired_delegators[1].conn_handle, paired_delegators[1].scan_control_point_h,
                                 &remote_scan_started, sizeof(remote_scan_started), NULL, NULL);
        }
    }
}

static void
set_security_data(void)
{
    ble_hs_cfg.sm_their_key_dist = 7;
    ble_hs_cfg.sm_our_key_dist = 7;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_KEYBOARD_DISPLAY;
}

static void
on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(1);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    assert(rc == 0);

    set_security_data();
}

static void
scan_btn_event_cb(lv_event_t *e)
{
    if (scanning) {
        stop_scan();
    } else {
        start_scan();
    }
}

static void
filter_btn_event_cb(lv_event_t *e)
{
    static uint8_t filter_btn_active = 0;

    if (filter_btn_active) {
        filter_btn_active = 0;
        lv_obj_move_background(gui.filters_list);
    } else {
        filter_btn_active = 1;
        lv_obj_move_foreground(gui.filters_list);
    }
}

static void
return_btn_event_cb(lv_event_t *e)
{
    int rc;
    rc = ble_gap_periodic_adv_sync_terminate(synced_source.sync_handle);
    printf("Periodic advertising stop rc = %d\n", rc);
    memset(&synced_source, 0, sizeof(synced_source));
    synced_source.list_id = LIST_ID_NONE;

    sort_suspended = 0;
    switch_to_devices_list();
}

static void
rssi_dropdown_event_cb(lv_event_t *e)
{
    char buf[4];
    lv_obj_t *dd = lv_event_get_target(e);

    lv_dropdown_get_selected_str(dd, buf, sizeof(buf));
    filter_settings.rssi = atoi(buf);
    printf("Minimum RSSI set: %d\n", filter_settings.rssi);
}

static void
hide_delegator_devices(void)
{
    uint8_t i;
    uint8_t n = lv_obj_get_child_cnt(gui.devices_list);

    for (i = 0; i < n; i++) {
        if (devices[i].device_role == DEVICE_ROLE_BROADCAST_DELEGATOR) {
            lv_obj_add_flag(devices[i].list_entry, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(devices[i].list_entry, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void
hide_source_devices(void)
{
    uint8_t i;
    uint8_t n = lv_obj_get_child_cnt(gui.devices_list);

    for (i = 0; i < n; i++) {
        if (devices[i].device_role == DEVICE_ROLE_BROADCAST_SOURCE) {
            lv_obj_add_flag(devices[i].list_entry, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(devices[i].list_entry, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void
device_role_dropdown_event_cb(lv_event_t *e)
{
    char buf[10];
    lv_obj_t *dd = lv_event_get_target(e);

    lv_dropdown_get_selected_str(dd, buf, sizeof(buf));

    if (!strcmp(buf, "Source")) {
        filter_settings.device_role = DEVICE_ROLE_BROADCAST_SOURCE;
        hide_delegator_devices();
    } else if (!strcmp(buf, "Delegator")) {
        filter_settings.device_role = DEVICE_ROLE_BROADCAST_DELEGATOR;
        hide_source_devices();
    } else {
        return;
    }

    printf("Scan set for device role: %s\n",
           filter_settings.device_role ? "Broadcast Sink" : "Broadcast Source");
}

static void
style_init(void)
{
    lv_style_init(&gui.style);
    lv_style_set_pad_ver(&gui.style, 0);
    lv_style_set_pad_hor(&gui.style, 2);
    lv_style_set_bg_color(&gui.style, lv_color_hex(COLOR_DEVICE));
}

static void
server_remove_source(struct paired_delegator *delegator)
{
    uint8_t data_to_send[2];

    data_to_send[0] = BROADCAST_ASSISTANT_REMOVE_SOURCE_OPCODE;
    data_to_send[1] = synced_source.server_id;

    ble_gattc_write_flat(delegator->conn_handle, delegator->scan_control_point_h, data_to_send, 2, NULL, NULL);
}

static void
remove_src_btn_event_cb(lv_event_t *e)
{
    if (paired_delegators[0].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        server_remove_source(&paired_delegators[0]);
    }
    if (paired_delegators[1].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        server_remove_source(&paired_delegators[1]);
    }
}

static void
btns_init(void)
{
    gui.scan_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(gui.scan_btn, 100, 50);
    lv_obj_align(gui.scan_btn, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_add_event_cb(gui.scan_btn, scan_btn_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(gui.scan_btn);
    lv_label_set_text(label, "Start\nscan");
    lv_obj_center(label);

    gui.filter_btn = lv_btn_create(lv_scr_act());
    lv_obj_set_size(gui.filter_btn, 100, 50);
    lv_obj_align(gui.filter_btn, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_obj_add_event_cb(gui.filter_btn, filter_btn_event_cb, LV_EVENT_CLICKED, NULL);

    label = lv_label_create(gui.filter_btn);
    lv_label_set_text(label, "Filter\noptions");
    lv_obj_center(label);

    gui.return_btn = lv_btn_create(lv_scr_act());
    lv_obj_add_flag(gui.return_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(gui.return_btn, 100, 50);
    lv_obj_align(gui.return_btn, LV_ALIGN_BOTTOM_RIGHT, -5, -5);
    lv_obj_add_event_cb(gui.return_btn, return_btn_event_cb, LV_EVENT_CLICKED, NULL);

    label = lv_label_create(gui.return_btn);
    lv_label_set_text(label, "Return");
    lv_obj_center(label);

    gui.remove_src_btn = lv_btn_create(lv_scr_act());
    lv_obj_add_flag(gui.remove_src_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(gui.remove_src_btn, 100, 50);
    lv_obj_align(gui.remove_src_btn, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    lv_obj_add_event_cb(gui.remove_src_btn, remove_src_btn_event_cb, LV_EVENT_CLICKED, NULL);

    label = lv_label_create(gui.remove_src_btn);
    lv_label_set_text(label, "Remove\nSource");
    lv_obj_center(label);
}

static void
filter_list_init(void)
{
    lv_obj_t *obj;

    gui.filters_list = lv_obj_create(lv_scr_act());
    lv_obj_set_size(gui.filters_list, LV_PCT(100), LV_PCT(80));
    lv_obj_align(gui.filters_list, LV_ALIGN_TOP_MID, 0, 5);

    lv_obj_set_flex_flow(gui.filters_list, LV_FLEX_FLOW_COLUMN);

    obj = lv_label_create(gui.filters_list);
    lv_label_set_text(obj, "Filter options");

    obj = lv_label_create(gui.filters_list);
    lv_label_set_text(obj, "Minimum RSSI:");

    obj = lv_dropdown_create(gui.filters_list);
    lv_dropdown_set_options(obj, "-75 dB\n" "-70 dB\n" "-65 dB\n" "-60 dB\n"
                                 "-55 dB\n" "-50 dB\n" "-45 dB\n" "-40 dB\n");
    lv_obj_add_event_cb(obj, rssi_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    obj = lv_label_create(gui.filters_list);
    lv_label_set_text(obj, "Scan for:");

    obj = lv_dropdown_create(gui.filters_list);
    lv_dropdown_set_options(obj, "Source\n" "Delegator\n");
    lv_obj_add_event_cb(obj, device_role_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

static void
display_init(void)
{
    style_init();
    filter_list_init();
    subgroups_list_init();
    devices_list_init();
    btns_init();
}

int
main(int argc, char **argv)
{
    /* Initialize all packages. */
    sysinit();

    display_init();

#if MYNEWT_VAL(LVGL_TRACKBALL)
    /* Enable trackball power supply gpio pin */
    hal_gpio_init_out(17,1);
#endif

    ble_hs_cfg.sync_cb = on_sync;

    /* As the last thing, process events from default event queue. */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}
