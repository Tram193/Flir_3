/**
 ********************************************************************
 * @file    file_binary_array_list_en.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "file_binary_array_list_en.h"

//#include "widget_file_c/en_big_screen/icon_button1_png.h"
//#include "widget_file_c/en_big_screen/icon_button2_png.h"
//#include "widget_file_c/en_big_screen/icon_list_item1_png.h"
//#include "widget_file_c/en_big_screen/icon_list_item2_png.h"
//#include "widget_file_c/en_big_screen/icon_scale_png.h"
//#include "widget_file_c/en_big_screen/icon_switch_select_png.h"
//#include "widget_file_c/en_big_screen/icon_switch_unselect_png.h"
//#include "widget_file_c/en_big_screen/widget_config_json.h"

#include "widget_file_c/en_big_screen/icon_list_security_png.h"
#include "widget_file_c/en_big_screen/icon_list_fullscreen_png.h"
#include "widget_file_c/en_big_screen/icon_list_pip_png.h"
#include "widget_file_c/en_big_screen/button_selected_thermal_png.h"
#include "widget_file_c/en_big_screen/button_selected_visible_png.h"
#include "widget_file_c/en_big_screen/icon_scale_png.h"
#include "widget_file_c/en_big_screen/widget_config_json.h"
#include "widget_file_c/en_big_screen/gimbal_on_item_png.h"
#include "widget_file_c/en_big_screen/gimbal_off_item_png.h"

/* Private constants ---------------------------------------------------------*/

/* Export types -------------------------------------------------------------*/
// English language file binary array list
static T_DjiWidgetFileBinaryArray s_EnWidgetFileBinaryArrayList[] = {
    {widget_config_json_fileName, widget_config_json_fileSize, widget_config_json_fileBinaryArray},

    {icon_list_fullscreen_png_fileName,     icon_list_fullscreen_png_fileSize,      icon_list_fullscreen_png_fileBinaryArray},
    {icon_list_security_png_fileName,       icon_list_security_png_fileSize,        icon_list_security_png_fileBinaryArray},
    {icon_list_pip_png_fileName,            icon_list_pip_png_fileSize,             icon_list_pip_png_fileBinaryArray},
    {button_selected_thermal_png_fileName,  button_selected_thermal_png_fileSize,   button_selected_thermal_png_fileBinaryArray},
    {button_selected_visible_png_fileName,  button_selected_visible_png_fileSize,   button_selected_visible_png_fileBinaryArray},
    {icon_scale_png_fileName,               icon_scale_png_fileSize,                icon_scale_png_fileBinaryArray},
    {gimbal_on_item_png_fileName,               gimbal_on_item_png_fileSize,                gimbal_on_item_png_fileBinaryArray},
    {gimbal_off_item_png_fileName,               gimbal_off_item_png_fileSize,                gimbal_off_item_png_fileBinaryArray},
};

/* Export values -------------------------------------------------------------*/
uint32_t g_testEnBinaryArrayCount = sizeof(s_EnWidgetFileBinaryArrayList) / sizeof(T_DjiWidgetFileBinaryArray);
T_DjiWidgetFileBinaryArray * g_testEnFileBinaryArrayList = s_EnWidgetFileBinaryArrayList;

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
