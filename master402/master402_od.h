
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef MASTER402_H
#define MASTER402_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 master402_valueRangeTest (UNS8 typeValue, void * value);
const indextable * master402_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data master402_Data;
extern UNS16 NODE3_Controlword_6040;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS16 NODE3_Statusword_6041;		/* Mapped at index 0x2002, subindex 0x00*/
extern INTEGER8 NODE3_Modes_of_operation_6060;		/* Mapped at index 0x2003, subindex 0x00*/
extern INTEGER32 NODE3_Position_actual_value_6064;		/* Mapped at index 0x2004, subindex 0x00*/
extern INTEGER32 NODE3_Velocity_actual_value_0x606C;		/* Mapped at index 0x2005, subindex 0x00*/
extern INTEGER32 NODE3_Target_position_607A;		/* Mapped at index 0x2006, subindex 0x00*/
extern INTEGER32 NODE3_Target_velocity_60FF;		/* Mapped at index 0x2007, subindex 0x00*/
extern UNS16 S_move;		/* Mapped at index 0x2124, subindex 0x00*/
extern INTEGER32 pos_cmd1;		/* Mapped at index 0x2F00, subindex 0x00*/
extern INTEGER16 pos_cmd2;		/* Mapped at index 0x2F01, subindex 0x00*/
extern UNS16 Controlword;		/* Mapped at index 0x6040, subindex 0x00*/
extern UNS16 Statusword;		/* Mapped at index 0x6041, subindex 0x00*/
extern INTEGER8 Modes_of_operation;		/* Mapped at index 0x6060, subindex 0x00*/
extern INTEGER32 Position_actual_value;		/* Mapped at index 0x6064, subindex 0x00*/
extern INTEGER32 Velocity_actual_value;		/* Mapped at index 0x606C, subindex 0x00*/
extern INTEGER32 Target_position;		/* Mapped at index 0x607A, subindex 0x00*/
extern INTEGER32 Home_offset;		/* Mapped at index 0x607C, subindex 0x00*/
extern UNS32 Profile_velocity;		/* Mapped at index 0x6081, subindex 0x00*/
extern INTEGER8 Homing_method;		/* Mapped at index 0x6098, subindex 0x00*/
extern UNS32 Homing_speeds_Speed_for_switch_search;		/* Mapped at index 0x6099, subindex 0x01 */
extern UNS32 Homing_speeds_Speed_for_zero_search;		/* Mapped at index 0x6099, subindex 0x02 */
extern INTEGER32 Interpolation_data_record_Parameter1_of_ip_function;		/* Mapped at index 0x60C1, subindex 0x01 */
extern UNS8 Interpolation_time_period_Interpolation_time_units;		/* Mapped at index 0x60C2, subindex 0x01 */
extern INTEGER8 Interpolation_time_period_Interpolation_time_index;		/* Mapped at index 0x60C2, subindex 0x02 */
extern INTEGER32 Target_velocity;		/* Mapped at index 0x60FF, subindex 0x00*/

#endif // MASTER402_H
