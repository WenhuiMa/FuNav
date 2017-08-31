#include "sensor.h"

using namespace amcl;

AMCLSensor::AMCLSensor()
{
  return;
}

AMCLSensor::~AMCLSensor()
{
}

bool AMCLSensor::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  return false;
}


bool AMCLSensor::InitSensor(pf_t *pf, AMCLSensorData *data)
{
  return false;
}


bool AMCLSensor::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  return false;
}

