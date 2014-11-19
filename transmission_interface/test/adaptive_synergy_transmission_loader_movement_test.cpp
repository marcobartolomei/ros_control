// adaptive_synergy_transmission_loader_movement_test.cpp

///////////////////////////////////////////////////////////////////////////////
//
// This file tests the function actuatorToJointPosition(); 
// By input will pass the values of the file urdf regarding the features,
// instead for the datu coming from the actuator, has been used a numerical constant location.
// On output is printed the value of joint_data.position, the other values of the joint data (velocity and effor) are set to zero.
//
//////////////////////////////////////////////////////////////////////////////

// Marco Bartolomei 2014

#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <transmission_interface/adaptive_synergy_transmission.h>
#include <transmission_interface/transmission_loader.h>
#include "read_file.h"
#include "loader_utils.h"


TEST(AdaptiveSynergyTransmissionLoaderTest, actTojnt_position_spec)
{

// CASE NO CONTACT

// Parse transmission info
  std::vector<TransmissionInfo> infos = parseUrdf("test/urdf/adaptive_synergy_transmission_loader_full.urdf");
  ASSERT_EQ(1, infos.size());

  // Transmission loader
  TransmissionPluginLoader loader;
  boost::shared_ptr<TransmissionLoader> transmission_loader = loader.create(infos.front().type_);
  
  ASSERT_TRUE(0 != transmission_loader);
  
  TransmissionPtr transmission;
  const TransmissionInfo& info = infos.front();
  
  transmission = transmission_loader->load(info);
  
  ASSERT_TRUE(0 != transmission);

  // Validate transmission
  AdaptiveSynergyTransmission* adaptive_synergy_transmission = dynamic_cast<AdaptiveSynergyTransmission*>(transmission.get());
  ASSERT_TRUE(0 != adaptive_synergy_transmission);

  const std::vector<double>& act_red = adaptive_synergy_transmission->getActuatorReduction(); // data loader jr
  
  const std::vector<double>& jnt_red = adaptive_synergy_transmission->getJointReduction();       // data loader ar
  
  // create a fixed actuator data for the test 
  ActuatorData act_data;

        *act_data.position[0] = 5;    // position;
        *act_data.velocity[0] = 0;    // velocity;
        *act_data.effort[0] = 0;      // effort;

       // printf("actuator value is: %f\n", *act_data.position[0]);

  // create a jointData null vector

  int num_jnt=20;      
  JointData jnt_data;

    // for (int i = 0; i < num_jnt; i++)
    // {
    
    //   *jnt_data.position[i] = 0;
    //   *jnt_data.velocity[i] = 0; 
    //   *jnt_data.effort[i] = 0; 
    // }
  
  // create an AdaptiveSynergyTransmission variable for upload the value
  


  AdaptiveSynergyTransmission* q_no_contact_solution;  // CASE NO CONTACT

 //q_no_contact_solution->actuatorToJointPosition(act_data, jnt_data);

   //const std::vector<double>& q_no_contact_solution = adaptive_synergy_transmission->actuatorToJointPosition(act_data, jnt_data);


  // set joint_data.position
     

      // // create a vector for saved the actuatotTojointPosistion data  ATTENTION 20 dimention
      // const std::vector<double>& q_no_contact_solution = std::vector<double>(20, 0.0); 

  // print screan the joint_data.position[i]
  // we consider 20 elements -> future we must modified to 19 elements

  for (int i = 0; i < num_jnt; i++)
  {
    
    //printf ("Joint n° %i : %d \n", i+1, jnt_data.position[i]);  // print element
  }

}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
