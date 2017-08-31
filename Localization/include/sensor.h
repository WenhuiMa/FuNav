#ifndef AMCL_SENSOR_H_MWH_20170831
#define AMCL_SENSOR_H_MWH_20170831

#include"particlefilter.h"

namespace amcl
{

// Forward declarations
class AMCLSensorData;


// Base class for all AMCL sensors
class AMCLSensor
{
  // Default constructor
  public: AMCLSensor();
         
  // Default destructor
  public: virtual ~AMCLSensor();

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  public: virtual bool UpdateAction(pf_t *pf, AMCLSensorData *data);

  // Initialize the filter based on the sensor model.  Returns true if the
  // filter has been initialized.
  public: virtual bool InitSensor(pf_t *pf, AMCLSensorData *data);

  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Flag is true if this is the action sensor
  public: bool is_action;

  // Action pose (action sensors only)
  public: pf_vector_t pose;

};



// Base class for all AMCL sensor measurements
class AMCLSensorData
{
  // Pointer to sensor that generated the data
  public: AMCLSensor *sensor;
          virtual ~AMCLSensorData() {}

  // Data timestamp
  public: double time;
};

}

#endif