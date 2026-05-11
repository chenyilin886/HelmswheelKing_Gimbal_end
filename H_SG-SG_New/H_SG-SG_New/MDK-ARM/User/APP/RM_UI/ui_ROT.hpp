//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_ROT_H
#define UI_ROT_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_ROT_now_figures[5];
extern uint8_t ui_ROT_dirty_figure[5];
extern ui_interface_string_t ui_ROT_now_strings[6];
extern uint8_t ui_ROT_dirty_string[6];

#define ui_ROT_Ungroup_Function2 ((ui_interface_rect_t*)&(ui_ROT_now_figures[0]))
#define ui_ROT_Ungroup_Vision ((ui_interface_round_t*)&(ui_ROT_now_figures[1]))
#define ui_ROT_Gimbal_Function1 ((ui_interface_rect_t*)&(ui_ROT_now_figures[2]))
#define ui_ROT_Chassis_SC ((ui_interface_arc_t*)&(ui_ROT_now_figures[3]))
#define ui_ROT_Chassis_Power ((ui_interface_arc_t*)&(ui_ROT_now_figures[4]))

#define ui_ROT_Ungroup_Super (&(ui_ROT_now_strings[0]))
#define ui_ROT_Ungroup_Fri (&(ui_ROT_now_strings[1]))
#define ui_ROT_Shoot_leave (&(ui_ROT_now_strings[2]))
#define ui_ROT_Gimbal_Only (&(ui_ROT_now_strings[3]))
#define ui_ROT_Gimbal_Follow (&(ui_ROT_now_strings[4]))
#define ui_ROT_Gimbal_Rotating (&(ui_ROT_now_strings[5]))

#ifdef MANUAL_DIRTY
#define ui_ROT_Ungroup_Function2_dirty (ui_ROT_dirty_figure[0])
#define ui_ROT_Ungroup_Vision_dirty (ui_ROT_dirty_figure[1])
#define ui_ROT_Gimbal_Function1_dirty (ui_ROT_dirty_figure[2])
#define ui_ROT_Chassis_SC_dirty (ui_ROT_dirty_figure[3])
#define ui_ROT_Chassis_Power_dirty (ui_ROT_dirty_figure[4])

#define ui_ROT_Ungroup_Super_dirty (ui_ROT_dirty_string[0])
#define ui_ROT_Ungroup_Fri_dirty (ui_ROT_dirty_string[1])
#define ui_ROT_Shoot_leave_dirty (ui_ROT_dirty_string[2])
#define ui_ROT_Gimbal_Only_dirty (ui_ROT_dirty_string[3])
#define ui_ROT_Gimbal_Follow_dirty (ui_ROT_dirty_string[4])
#define ui_ROT_Gimbal_Rotating_dirty (ui_ROT_dirty_string[5])
#endif

void ui_init_ROT();
void ui_update_ROT();

#endif // UI_ROT_H
