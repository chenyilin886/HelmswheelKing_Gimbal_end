//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_ROT.h"

#define TOTAL_FIGURE 5
#define TOTAL_STRING 6

ui_interface_figure_t ui_ROT_now_figures[TOTAL_FIGURE];
uint8_t ui_ROT_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_now_strings[TOTAL_STRING];
uint8_t ui_ROT_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_ROT_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_ROT_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_ROT() {
    ui_ROT_Ungroup_Function2->figure_type = 1;
    ui_ROT_Ungroup_Function2->operate_type = 1;
    ui_ROT_Ungroup_Function2->layer = 0;
    ui_ROT_Ungroup_Function2->color = 8;
    ui_ROT_Ungroup_Function2->start_x = 761;
    ui_ROT_Ungroup_Function2->start_y = 170;
    ui_ROT_Ungroup_Function2->width = 1;
    ui_ROT_Ungroup_Function2->end_x = 1161;
    ui_ROT_Ungroup_Function2->end_y = 270;

    ui_ROT_Ungroup_Vision->figure_type = 2;
    ui_ROT_Ungroup_Vision->operate_type = 1;
    ui_ROT_Ungroup_Vision->layer = 0;
    ui_ROT_Ungroup_Vision->color = 0;
    ui_ROT_Ungroup_Vision->start_x = 959;
    ui_ROT_Ungroup_Vision->start_y = 535;
    ui_ROT_Ungroup_Vision->width = 10;
    ui_ROT_Ungroup_Vision->r = 75;

    ui_ROT_Gimbal_Function1->figure_type = 1;
    ui_ROT_Gimbal_Function1->operate_type = 1;
    ui_ROT_Gimbal_Function1->layer = 0;
    ui_ROT_Gimbal_Function1->color = 8;
    ui_ROT_Gimbal_Function1->start_x = 661;
    ui_ROT_Gimbal_Function1->start_y = 70;
    ui_ROT_Gimbal_Function1->width = 1;
    ui_ROT_Gimbal_Function1->end_x = 1261;
    ui_ROT_Gimbal_Function1->end_y = 170;

    ui_ROT_Chassis_SC->figure_type = 4;
    ui_ROT_Chassis_SC->operate_type = 1;
    ui_ROT_Chassis_SC->layer = 0;
    ui_ROT_Chassis_SC->color = 3;
    ui_ROT_Chassis_SC->start_x = 854;
    ui_ROT_Chassis_SC->start_y = 539;
    ui_ROT_Chassis_SC->width = 1;
    ui_ROT_Chassis_SC->start_angle = 220;
    ui_ROT_Chassis_SC->end_angle = 320;
    ui_ROT_Chassis_SC->rx = 286;
    ui_ROT_Chassis_SC->ry = 352;

    ui_ROT_Chassis_Power->figure_type = 4;
    ui_ROT_Chassis_Power->operate_type = 1;
    ui_ROT_Chassis_Power->layer = 0;
    ui_ROT_Chassis_Power->color = 0;
    ui_ROT_Chassis_Power->start_x = 1064;
    ui_ROT_Chassis_Power->start_y = 539;
    ui_ROT_Chassis_Power->width = 1;
    ui_ROT_Chassis_Power->start_angle = 40;
    ui_ROT_Chassis_Power->end_angle = 140;
    ui_ROT_Chassis_Power->rx = 286;
    ui_ROT_Chassis_Power->ry = 352;

    ui_ROT_Ungroup_Super->figure_type = 7;
    ui_ROT_Ungroup_Super->operate_type = 1;
    ui_ROT_Ungroup_Super->layer = 1;
    ui_ROT_Ungroup_Super->color = 2;
    ui_ROT_Ungroup_Super->start_x = 830;
    ui_ROT_Ungroup_Super->start_y = 221;
    ui_ROT_Ungroup_Super->width = 2;
    ui_ROT_Ungroup_Super->font_size = 20;
    ui_ROT_Ungroup_Super->str_length = 3;
    strcpy(ui_ROT_Ungroup_Super->string, "Sup");

    ui_ROT_Ungroup_Fri->figure_type = 7;
    ui_ROT_Ungroup_Fri->operate_type = 1;
    ui_ROT_Ungroup_Fri->layer = 1;
    ui_ROT_Ungroup_Fri->color = 2;
    ui_ROT_Ungroup_Fri->start_x = 1031;
    ui_ROT_Ungroup_Fri->start_y = 221;
    ui_ROT_Ungroup_Fri->width = 2;
    ui_ROT_Ungroup_Fri->font_size = 20;
    ui_ROT_Ungroup_Fri->str_length = 3;
    strcpy(ui_ROT_Ungroup_Fri->string, "Fri");

    ui_ROT_Shoot_leave->figure_type = 7;
    ui_ROT_Shoot_leave->operate_type = 1;
    ui_ROT_Shoot_leave->layer = 1;
    ui_ROT_Shoot_leave->color = 8;
    ui_ROT_Shoot_leave->start_x = 1456;
    ui_ROT_Shoot_leave->start_y = 735;
    ui_ROT_Shoot_leave->width = 3;
    ui_ROT_Shoot_leave->font_size = 26;
    ui_ROT_Shoot_leave->str_length = 12;
    strcpy(ui_ROT_Shoot_leave->string, "已发射：");

    ui_ROT_Gimbal_Only->figure_type = 7;
    ui_ROT_Gimbal_Only->operate_type = 1;
    ui_ROT_Gimbal_Only->layer = 1;
    ui_ROT_Gimbal_Only->color = 2;
    ui_ROT_Gimbal_Only->start_x = 730;
    ui_ROT_Gimbal_Only->start_y = 126;
    ui_ROT_Gimbal_Only->width = 2;
    ui_ROT_Gimbal_Only->font_size = 22;
    ui_ROT_Gimbal_Only->str_length = 3;
    strcpy(ui_ROT_Gimbal_Only->string, "ONE");

    ui_ROT_Gimbal_Follow->figure_type = 7;
    ui_ROT_Gimbal_Follow->operate_type = 1;
    ui_ROT_Gimbal_Follow->layer = 1;
    ui_ROT_Gimbal_Follow->color = 2;
    ui_ROT_Gimbal_Follow->start_x = 1130;
    ui_ROT_Gimbal_Follow->start_y = 124;
    ui_ROT_Gimbal_Follow->width = 2;
    ui_ROT_Gimbal_Follow->font_size = 22;
    ui_ROT_Gimbal_Follow->str_length = 3;
    strcpy(ui_ROT_Gimbal_Follow->string, "FOL");

    ui_ROT_Gimbal_Rotating->figure_type = 7;
    ui_ROT_Gimbal_Rotating->operate_type = 1;
    ui_ROT_Gimbal_Rotating->layer = 1;
    ui_ROT_Gimbal_Rotating->color = 2;
    ui_ROT_Gimbal_Rotating->start_x = 930;
    ui_ROT_Gimbal_Rotating->start_y = 124;
    ui_ROT_Gimbal_Rotating->width = 2;
    ui_ROT_Gimbal_Rotating->font_size = 22;
    ui_ROT_Gimbal_Rotating->str_length = 3;
    strcpy(ui_ROT_Gimbal_Rotating->string, "ROT");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
#endif
        ui_ROT_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_ROT_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_ROT_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_ROT_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
#endif
        ui_ROT_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_ROT_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_ROT_now_strings[i].operate_type = 2;
    }
}

void ui_update_ROT() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_ROT_now_figures[i], &ui_ROT_last_figures[i], sizeof(ui_ROT_now_figures[i])) != 0) {
            ui_ROT_dirty_figure[i] = 1;
            ui_ROT_last_figures[i] = ui_ROT_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_ROT_now_strings[i], &ui_ROT_last_strings[i], sizeof(ui_ROT_now_strings[i])) != 0) {
            ui_ROT_dirty_string[i] = 1;
            ui_ROT_last_strings[i] = ui_ROT_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
