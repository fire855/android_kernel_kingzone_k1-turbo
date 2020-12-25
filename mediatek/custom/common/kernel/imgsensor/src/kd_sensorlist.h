//s_add new sensor driver here
//export funtions
UINT32 IMX135_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 IMX219_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 IMX111_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
UINT32 OV5647MIPISensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);

//! Add Sensor Init function here
//! Note:
//! 1. Add by the resolution from ""large to small"", due to large sensor
//!    will be possible to be main sensor.
//!    This can avoid I2C error during searching sensor.
//! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR+1] =
{
#if defined(IMX135_MIPI_RAW)
    {IMX135_SENSOR_ID, SENSOR_DRVNAME_IMX135_MIPI_RAW, IMX135_MIPI_RAW_SensorInit},
#endif
#if defined(IMX219_MIPI_RAW)
    {IMX219_SENSOR_ID, SENSOR_DRVNAME_IMX219_MIPI_RAW, IMX219_MIPI_RAW_SensorInit},
#endif
#if defined(IMX111_MIPI_RAW)
    {IMX111_SENSOR_ID, SENSOR_DRVNAME_IMX111_MIPI_RAW, IMX111_MIPI_RAW_SensorInit},
#endif
#if defined(OV5647_MIPI_RAW)
    {OV5647MIPI_SENSOR_ID, SENSOR_DRVNAME_OV5647MIPI_RAW, OV5647MIPISensorInit}, 
#endif

/*  ADD sensor driver before this line */
    {0,{0},NULL}, //end of list
};
//e_add new sensor driver here
