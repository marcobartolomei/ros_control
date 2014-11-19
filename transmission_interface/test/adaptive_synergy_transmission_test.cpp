// file: adaptive_synergy_transmission_test.cpp


///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian
/// \ modified by Marco Bartolomei 2014

#include <gtest/gtest.h>

#include <transmission_interface/adaptive_synergy_transmission.h>
#include "random_generator_utils.h"

using namespace transmission_interface;
using std::vector;

// Floating-point value comparison threshold
const double EPS = 1e-6;


TEST(PreconditionsTest, ExceptionThrowing)
{

  // AdaptiveSynergyTransmission struct --> 4 parameters
  /* AdaptiveSynergyTransmission(const std::vector<double>& actuator_reduction,                             1)
                                 const std::vector<double>& joint_reduction,                                2)
                                 const std::vector<double>& joint_elastic,                                  3)
                                 const std::vector<double>& joint_offset = std::vector<double>(19, 0.0));   4)
                                 */

  // actuator reduction                              
  std::vector<double> act_red_good(1, 1.0);
  std::vector<double> act_red_bad(1, 0.0);
  
  // joint reduction
  std::vector<double> jnt_red_good(19, 1.0);
  std::vector<double> jnt_red_bad1(19, 0.0); 
  std::vector<double> jnt_red_bad2(19, 0.0); jnt_red_bad2[0] = 1.0; jnt_red_bad2[10] = 1.0; // 2/19 good values
  std::vector<double> jnt_red_bad3(19, 0.0); jnt_red_bad3[1] = 1.0; jnt_red_bad3[11] = 1.0; jnt_red_bad3[6] = 1.0; // 3/19 good values

  // joint elastic
  std::vector<double> jnt_els_good(19, 1.0);
  std::vector<double> jnt_els_bad1(19, 0.0); 
  std::vector<double> jnt_els_bad2(19, 0.0); jnt_els_bad2[0] = 1.0; jnt_els_bad2[10] = 1.0; // 2/19 good values
  std::vector<double> jnt_els_bad3(19, 0.0); jnt_els_bad3[1] = 1.0; jnt_els_bad3[11] = 1.0; jnt_els_bad3[6] = 1.0; // 3/19 good values

  // joint offset
  std::vector<double> offset_good(19, 1.0);


  // Invalid instance creation: Transmission cannot have zero reduction or elastic
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad1), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad2), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad3), TransmissionInterfaceException);

  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad1,jnt_els_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad2,jnt_els_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad3,jnt_els_good), TransmissionInterfaceException);

  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_bad,jnt_red_good,jnt_els_good), TransmissionInterfaceException);


  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad1,offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad2,offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_good,jnt_els_bad3,offset_good), TransmissionInterfaceException);

  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad1,jnt_els_good,offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad2,jnt_els_good,offset_good), TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good,jnt_red_bad3,jnt_els_good,offset_good), TransmissionInterfaceException);

  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_bad,jnt_red_good,jnt_els_good,offset_good), TransmissionInterfaceException);



  // Invalid instance creation: Wrong parameter sizes
  std::vector<double> act_red_bad_size(2, 1.0);    // excess size
  std::vector<double> jnt_bad_size(15, 1.0);       // reduced size
  std::vector<double>& offset_bad_size = jnt_bad_size;
  
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_bad_size, jnt_red_good, jnt_els_good), TransmissionInterfaceException);


  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_bad_size, jnt_bad_size),   TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_bad_size, jnt_els_good),   TransmissionInterfaceException);
  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_bad_size, jnt_bad_size),   TransmissionInterfaceException);


  EXPECT_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_red_good, jnt_els_good, offset_bad_size), TransmissionInterfaceException);

  // Valid instance creation
  EXPECT_NO_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_red_good, jnt_els_good));
  EXPECT_NO_THROW(AdaptiveSynergyTransmission(act_red_good, jnt_red_good, jnt_els_good, offset_good));
}


#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(PreconditionsTest, AssertionTriggering)
{

  // Create input/output transmission data
  
  double a_good_val = 0.0;      // actuator good value
  std::vector<double> j_good_val = std::vector<double>(19, 0.0);      // set to zero alla values
  
  std::vector<double*> a_good_vec;
  a_good_vec.push_back(&a_good_val);

  std::vector<double*> j_good_vec;

  for (int i = 0; i < 19; i++)
  {
    j_good_vec.push_back(&j_good_val[i]);
  }
  

  ActuatorData a_good_data;
  a_good_data.position = a_good_vec;
  a_good_data.velocity = a_good_vec;
  a_good_data.effort   = a_good_vec;

  JointData j_good_data;
  j_good_data.position = j_good_vec;
  j_good_data.velocity = j_good_vec;
  j_good_data.effort   = j_good_vec;

  ActuatorData a_bad_data;
  a_bad_data.position = std::vector<double*>(1);
  a_bad_data.velocity = std::vector<double*>(1);
  a_bad_data.effort   = std::vector<double*>(1);

  JointData j_bad_data;
  j_bad_data.position = std::vector<double*>(19);
  j_bad_data.velocity = std::vector<double*>(19);
  j_bad_data.effort   = std::vector<double*>(19);

  ActuatorData a_bad_size;
  JointData    j_bad_size;

  // Transmission instance
  AdaptiveSynergyTransmission trans(std::vector<double>(1, 1.0),
                                    std::vector<double>(19, 1.0),
                                    std::vector<double>(19, 1.0),
                                    std::vector<double>(19, 0.0));

  // Data with invalid pointers should trigger an assertion

  // actuarorTo.... 
    EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_data,  j_good_data), ".*");

    EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_data,  j_good_data), ".*");

    EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_data),  ".*");
    EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_data,  j_good_data), ".*");

  // jointTo...
    EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_data,  a_good_data), ".*");

    EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_data,  a_good_data), ".*");


    EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_data),  ".*");
    EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_data,  a_good_data), ".*");

  // Wrong parameter SIZES should trigger an assertion

  // actuarorTo....
    EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointEffort(a_good_data, j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointEffort(a_bad_size,  j_good_data), ".*");

    EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointVelocity(a_good_data, j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointVelocity(a_bad_size,  j_good_data), ".*");

    EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointPosition(a_good_data, j_bad_size),  ".*");
    EXPECT_DEATH(trans.actuatorToJointPosition(a_bad_size,  j_good_data), ".*");

  // jointTo...
    EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorEffort(j_good_data, a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorEffort(j_bad_size,  a_good_data), ".*");

    EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorVelocity(j_good_data, a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorVelocity(j_bad_size,  a_good_data), ".*");

    EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorPosition(j_good_data, a_bad_size),  ".*");
    EXPECT_DEATH(trans.jointToActuatorPosition(j_bad_size,  a_good_data), ".*");

}
#endif // NDEBUG

TEST(PreconditionsTest, AccessorValidation)
{
  
  std::vector<double> act_reduction(1);
  act_reduction[0] =  1.0;

  std::vector<double> jnt_reduction(19);
  jnt_reduction[0]  =  2.0;
  jnt_reduction[1]  =  2.0;
  jnt_reduction[2]  =  2.0;
  jnt_reduction[3]  =  2.0;
  jnt_reduction[4]  =  2.0;
  jnt_reduction[5]  =  2.0;
  jnt_reduction[6]  =  2.0;
  jnt_reduction[7]  =  2.0;
  jnt_reduction[8]  =  2.0;
  jnt_reduction[9]  =  2.0;
  jnt_reduction[10] =  2.0;
  jnt_reduction[11] =  2.0;
  jnt_reduction[12] =  2.0;
  jnt_reduction[13] =  2.0;
  jnt_reduction[14] =  2.0;
  jnt_reduction[15] =  2.0;
  jnt_reduction[16] =  2.0;
  jnt_reduction[17] =  2.0;
  jnt_reduction[18] =  2.0;
  

  std::vector<double> jnt_elastic(19);
  jnt_elastic[0]  =  4.0;
  jnt_elastic[1]  =  4.0;
  jnt_elastic[2]  =  4.0;
  jnt_elastic[3]  =  4.0;
  jnt_elastic[4]  =  4.0;
  jnt_elastic[5]  =  4.0;
  jnt_elastic[6]  =  4.0;
  jnt_elastic[7]  =  4.0;
  jnt_elastic[8]  =  4.0;
  jnt_elastic[9]  =  4.0;
  jnt_elastic[10] =  4.0;
  jnt_elastic[11] =  4.0;
  jnt_elastic[12] =  4.0;
  jnt_elastic[13] =  4.0;
  jnt_elastic[14] =  4.0;
  jnt_elastic[15] =  4.0;
  jnt_elastic[16] =  4.0;
  jnt_elastic[17] =  4.0;
  jnt_elastic[18] =  4.0;

  std::vector<double> jnt_offset(19);
  jnt_offset[0] =  0.0;
  jnt_offset[1] =  0.0;
  jnt_offset[2] =  0.0;
  jnt_offset[3] =  0.0; 
  jnt_offset[4] =  0.0;
  jnt_offset[5] =  0.0; 
  jnt_offset[6] =  0.0;
  jnt_offset[7] =  0.0; 
  jnt_offset[8] =  0.0;
  jnt_offset[9] =  0.0;
  jnt_offset[10] =  0.0;
  jnt_offset[11] =  0.0;
  jnt_offset[12] =  0.0;
  jnt_offset[13] =  0.0; 
  jnt_offset[14] =  0.0;
  jnt_offset[15] =  0.0; 
  jnt_offset[16] =  0.0;
  jnt_offset[17] =  0.0; 
  jnt_offset[18] =  0.0;


  AdaptiveSynergyTransmission trans(act_reduction, jnt_reduction, jnt_elastic, jnt_offset);

  EXPECT_EQ(1, trans.numActuators());
  EXPECT_EQ(19, trans.numJoints());

  // actuator reduction
  EXPECT_EQ( 1.0, trans.getActuatorReduction()[0]);

  // joint reduction
  EXPECT_EQ( 2.0, trans.getJointReduction()[0]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[1]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[2]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[3]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[4]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[5]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[6]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[7]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[8]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[9]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[10]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[11]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[12]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[13]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[14]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[15]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[16]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[17]);
  EXPECT_EQ( 2.0, trans.getJointReduction()[18]);

  // joint elastic
  EXPECT_EQ(4.0, trans.getJointElastic()[0]);
  EXPECT_EQ(4.0, trans.getJointElastic()[1]);
  EXPECT_EQ(4.0, trans.getJointElastic()[2]);
  EXPECT_EQ(4.0, trans.getJointElastic()[3]);
  EXPECT_EQ(4.0, trans.getJointElastic()[4]);
  EXPECT_EQ(4.0, trans.getJointElastic()[5]);
  EXPECT_EQ(4.0, trans.getJointElastic()[6]);
  EXPECT_EQ(4.0, trans.getJointElastic()[7]);
  EXPECT_EQ(4.0, trans.getJointElastic()[8]);
  EXPECT_EQ(4.0, trans.getJointElastic()[9]);
  EXPECT_EQ(4.0, trans.getJointElastic()[10]);
  EXPECT_EQ(4.0, trans.getJointElastic()[11]);
  EXPECT_EQ(4.0, trans.getJointElastic()[12]);
  EXPECT_EQ(4.0, trans.getJointElastic()[13]);
  EXPECT_EQ(4.0, trans.getJointElastic()[14]);
  EXPECT_EQ(4.0, trans.getJointElastic()[15]);
  EXPECT_EQ(4.0, trans.getJointElastic()[16]);
  EXPECT_EQ(4.0, trans.getJointElastic()[17]);
  EXPECT_EQ(4.0, trans.getJointElastic()[18]);

  // joint offset
  EXPECT_EQ( 0.0, trans.getJointOffset()[0]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[1]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[2]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[3]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[4]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[5]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[6]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[7]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[8]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[9]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[10]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[11]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[12]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[13]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[14]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[15]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[16]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[17]);
  EXPECT_EQ( 0.0, trans.getJointOffset()[18]);

}


class TransmissionSetup : public ::testing::Test
{
public:
  TransmissionSetup()
    : a_val(),
      j_val(),
      a_vec(std::vector<double*>(1)),
      j_vec(std::vector<double*>(19))
   {
     a_vec[0] = &a_val[0];
     
     j_vec[0]  = &j_val[0];
     j_vec[1]  = &j_val[1];
     j_vec[2]  = &j_val[2];
     j_vec[3]  = &j_val[3];
     j_vec[4]  = &j_val[4];
     j_vec[5]  = &j_val[5];
     j_vec[6]  = &j_val[6];
     j_vec[7]  = &j_val[7];
     j_vec[8]  = &j_val[8];
     j_vec[9]  = &j_val[9];
     j_vec[10] = &j_val[10];
     j_vec[11] = &j_val[11];
     j_vec[12] = &j_val[12];
     j_vec[13] = &j_val[13];
     j_vec[14] = &j_val[14];
     j_vec[15] = &j_val[15];
     j_vec[16] = &j_val[16];
     j_vec[17] = &j_val[17];
     j_vec[18] = &j_val[18];

   }

protected:
  // Input/output transmission data
  double a_val[1];
  double j_val[19];
  std::vector<double*> a_vec;
  std::vector<double*> j_vec;
};

/// \brief Exercises the actuator->joint->actuator roundtrip, which should yield the identity map.
class BlackBoxTest : public TransmissionSetup
{
protected:
  // \param trans Transmission instance.
  // \param ref_val Reference value (effort, velocity or position) that will be transformed with the respective forward
  // and inverse transmission transformations.


  // void testIdentityMap(AdaptiveSynergyTransmission& trans,
  //                      const vector<double>& ref_val)
  // {
  //   // Effort interface
  //   {
  //     // actuator effort
  //     ActuatorData a_data;
  //     a_data.effort = a_vec;
  //     *a_data.effort[0] = ref_val[0];

  //     // joint effort
  //     JointData j_data;
  //     j_data.effort = j_vec;

  //     trans.actuatorToJointEffort(a_data, j_data);
  //     trans.jointToActuatorEffort(j_data, a_data);
  //     EXPECT_NEAR(ref_val[0], *a_data.effort[0], EPS);
  //   }

  //   // Velocity interface
  //   {
  //     // actuator velocity
  //     ActuatorData a_data;
  //     a_data.velocity = a_vec;
  //     *a_data.velocity[0] = ref_val[0];
      
  //     // joint velocity
  //     JointData j_data;
  //     j_data.velocity = j_vec;

  //     trans.actuatorToJointVelocity(a_data, j_data);
  //     trans.jointToActuatorVelocity(j_data, a_data);
  //     EXPECT_NEAR(ref_val[0], *a_data.velocity[0], EPS);
  //   }

  //   // Position interface
  //   {
  //     // actuator position
  //     ActuatorData a_data;
  //     a_data.position = a_vec;
  //     *a_data.position[0] = ref_val[0];

  //     // joint position
  //     JointData j_data;
  //     j_data.position = j_vec;

  //     trans.actuatorToJointPosition(a_data, j_data);
  //     trans.jointToActuatorPosition(j_data, a_data);
  //     EXPECT_NEAR(ref_val[0], *a_data.position[0], EPS);
  //   }
  // }



  /// Generate a set of transmission instances with random combinations of actuator/joint reduction and joint offset.
  static std::vector<AdaptiveSynergyTransmission> createTestInstances(const std::vector<AdaptiveSynergyTransmission>::size_type size)
  {
    std::vector<AdaptiveSynergyTransmission> out;
    out.reserve(size);
    RandomDoubleGenerator rand_gen(-1.0, 1.0);                                // NOTE: Magic value

    while (out.size() < size)
    {
      try
      {
        AdaptiveSynergyTransmission trans(randomVector(1,  rand_gen),
                                          randomVector(19, rand_gen),
                                          randomVector(19, rand_gen),
                                          randomVector(19, rand_gen));
        out.push_back(trans);
      }
      catch(const TransmissionInterfaceException&)
      {
        // NOTE: If by chance a perfect zero is produced by the random number generator, construction will fail
        // We swallow the exception and move on to prevent a test crash.
      }
    }
    return out;
  }
};



class WhiteBoxTest : public TransmissionSetup {};

TEST_F(WhiteBoxTest, DontMoveJoints)
{


  std::vector<double> act_reduction(1, 1.0);
  std::vector<double> jnt_reduction(19, 2.0);
  std::vector<double> jnt_elastic(19, 4.0);
  std::vector<double> jnt_offset(19, 0.0);
  
  AdaptiveSynergyTransmission trans(act_reduction, jnt_reduction, jnt_elastic, jnt_offset);


  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 0.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;
    //a_data.effort[0] = a_vec[0];

    JointData j_data;
    j_data.effort = j_vec;
    j_data.position = j_vec;


    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[1], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[2], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[3], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[4], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[5], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[6], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[7], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[8], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[9], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[10], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[11], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[12], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[13], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[14], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[15], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[16], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[17], EPS);
    EXPECT_NEAR(0.0, *j_data.effort[18], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.velocity[0], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[1], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[2], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[3], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[4], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[5], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[6], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[7], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[8], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[9], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[10], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[11], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[12], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[13], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[14], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[15], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[16], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[17], EPS);
    EXPECT_NEAR(0.0, *j_data.velocity[18], EPS);  
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(0.0, *j_data.position[0], EPS);
    EXPECT_NEAR(0.0, *j_data.position[1], EPS);
    EXPECT_NEAR(0.0, *j_data.position[2], EPS);
    EXPECT_NEAR(0.0, *j_data.position[3], EPS);
    EXPECT_NEAR(0.0, *j_data.position[4], EPS);
    EXPECT_NEAR(0.0, *j_data.position[5], EPS);
    EXPECT_NEAR(0.0, *j_data.position[6], EPS);
    EXPECT_NEAR(0.0, *j_data.position[7], EPS);
    EXPECT_NEAR(0.0, *j_data.position[8], EPS);
    EXPECT_NEAR(0.0, *j_data.position[9], EPS);
    EXPECT_NEAR(0.0, *j_data.position[10], EPS);
    EXPECT_NEAR(0.0, *j_data.position[11], EPS);
    EXPECT_NEAR(0.0, *j_data.position[12], EPS);
    EXPECT_NEAR(0.0, *j_data.position[13], EPS);
    EXPECT_NEAR(0.0, *j_data.position[14], EPS);
    EXPECT_NEAR(0.0, *j_data.position[15], EPS);
    EXPECT_NEAR(0.0, *j_data.position[16], EPS);
    EXPECT_NEAR(0.0, *j_data.position[17], EPS);
    EXPECT_NEAR(0.0, *j_data.position[18], EPS);
  }
  
}


TEST_F(WhiteBoxTest, MoveHand)
{
  // NOTE: We only test the actuator->joint map, as the joint->actuator map is indirectly validated in the test that
  // checks that actuator->joint->actuator == identity.
  // we move all joint of soft-hand

  vector<double> act_reduction(1);
  act_reduction[0] =  1.0;

  vector<double> jnt_reduction(19);
  jnt_reduction[0]  =  2.0;
  jnt_reduction[1]  =  2.0;
  jnt_reduction[2]  =  2.0;
  jnt_reduction[3]  =  2.0;
  jnt_reduction[4]  =  2.0;
  jnt_reduction[5]  =  2.0;
  jnt_reduction[6]  =  2.0;
  jnt_reduction[7]  =  2.0;
  jnt_reduction[8]  =  2.0;
  jnt_reduction[9]  =  2.0;
  jnt_reduction[10] =  2.0;
  jnt_reduction[11] =  2.0;
  jnt_reduction[12] =  2.0;
  jnt_reduction[13] =  2.0;
  jnt_reduction[14] =  2.0;
  jnt_reduction[15] =  2.0;
  jnt_reduction[16] =  2.0;
  jnt_reduction[17] =  2.0;
  jnt_reduction[18] =  2.0;
  

  vector<double> jnt_elastic(19);
  jnt_elastic[0]  =  4.0;
  jnt_elastic[1]  =  4.0;
  jnt_elastic[2]  =  4.0;
  jnt_elastic[3]  =  4.0;
  jnt_elastic[4]  =  4.0;
  jnt_elastic[5]  =  4.0;
  jnt_elastic[6]  =  4.0;
  jnt_elastic[7]  =  4.0;
  jnt_elastic[8]  =  4.0;
  jnt_elastic[9]  =  4.0;
  jnt_elastic[10] =  4.0;
  jnt_elastic[11] =  4.0;
  jnt_elastic[12] =  4.0;
  jnt_elastic[13] =  4.0;
  jnt_elastic[14] =  4.0;
  jnt_elastic[15] =  4.0;
  jnt_elastic[16] =  4.0;
  jnt_elastic[17] =  4.0;
  jnt_elastic[18] =  4.0;

  vector<double> jnt_offset(19);
  jnt_offset[0] =  0.0;
  jnt_offset[1] =  0.0;
  jnt_offset[2] =  0.0;
  jnt_offset[3] =  0.0; 
  jnt_offset[4] =  0.0;
  jnt_offset[5] =  0.0; 
  jnt_offset[6] =  0.0;
  jnt_offset[7] =  0.0; 
  jnt_offset[8] =  0.0;
  jnt_offset[9] =  0.0;
  jnt_offset[10] =  0.0;
  jnt_offset[11] =  0.0;
  jnt_offset[12] =  0.0;
  jnt_offset[13] =  0.0; 
  jnt_offset[14] =  0.0;
  jnt_offset[15] =  0.0; 
  jnt_offset[16] =  0.0;
  jnt_offset[17] =  0.0; 
  jnt_offset[18] =  0.0;


  AdaptiveSynergyTransmission trans(act_reduction, jnt_reduction, jnt_elastic, jnt_offset);

  // Actuator input (used for effort, velocity and position)
  *a_vec[0] = 3.0;

  // Effort interface
  {
    ActuatorData a_data;
    a_data.effort = a_vec;

    JointData j_data;
    j_data.effort = j_vec;
    j_data.position = j_vec;


    trans.actuatorToJointEffort(a_data, j_data);
    EXPECT_NEAR(6.0, *j_data.effort[0], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[1], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[2], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[3], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[4], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[5], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[6], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[7], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[8], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[9], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[10], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[11], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[12], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[13], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[14], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[15], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[16], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[17], EPS);
    EXPECT_NEAR(6.0, *j_data.effort[18], EPS);
  }

  // Velocity interface
  {
    ActuatorData a_data;
    a_data.velocity = a_vec;

    JointData j_data;
    j_data.velocity = j_vec;

    trans.actuatorToJointVelocity(a_data, j_data);
    EXPECT_NEAR(6, *j_data.velocity[0], EPS);
    EXPECT_NEAR(6, *j_data.velocity[1], EPS);
    EXPECT_NEAR(6, *j_data.velocity[2], EPS);
    EXPECT_NEAR(6, *j_data.velocity[3], EPS);
    EXPECT_NEAR(6, *j_data.velocity[4], EPS);
    EXPECT_NEAR(6, *j_data.velocity[5], EPS);
    EXPECT_NEAR(6, *j_data.velocity[6], EPS);
    EXPECT_NEAR(6, *j_data.velocity[7], EPS);
    EXPECT_NEAR(6, *j_data.velocity[8], EPS);
    EXPECT_NEAR(6, *j_data.velocity[9], EPS);
    EXPECT_NEAR(6, *j_data.velocity[10], EPS);
    EXPECT_NEAR(6, *j_data.velocity[11], EPS);
    EXPECT_NEAR(6, *j_data.velocity[12], EPS);
    EXPECT_NEAR(6, *j_data.velocity[13], EPS);
    EXPECT_NEAR(6, *j_data.velocity[14], EPS);
    EXPECT_NEAR(6, *j_data.velocity[15], EPS);
    EXPECT_NEAR(6, *j_data.velocity[16], EPS);
    EXPECT_NEAR(6, *j_data.velocity[17], EPS);
    EXPECT_NEAR(6, *j_data.velocity[18], EPS);
  }

  // Position interface
  {
    ActuatorData a_data;
    a_data.position = a_vec;

    JointData j_data;
    j_data.position = j_vec;

    trans.actuatorToJointPosition(a_data, j_data);
    EXPECT_NEAR(6, *j_data.position[0], EPS);
    EXPECT_NEAR(6, *j_data.position[1], EPS);
    EXPECT_NEAR(6, *j_data.position[2], EPS);
    EXPECT_NEAR(6, *j_data.position[3], EPS);
    EXPECT_NEAR(6, *j_data.position[4], EPS);
    EXPECT_NEAR(6, *j_data.position[5], EPS);
    EXPECT_NEAR(6, *j_data.position[6], EPS);
    EXPECT_NEAR(6, *j_data.position[7], EPS);
    EXPECT_NEAR(6, *j_data.position[8], EPS);
    EXPECT_NEAR(6, *j_data.position[9], EPS);
    EXPECT_NEAR(6, *j_data.position[10], EPS);
    EXPECT_NEAR(6, *j_data.position[11], EPS);
    EXPECT_NEAR(6, *j_data.position[12], EPS);
    EXPECT_NEAR(6, *j_data.position[13], EPS);
    EXPECT_NEAR(6, *j_data.position[14], EPS);
    EXPECT_NEAR(6, *j_data.position[15], EPS);
    EXPECT_NEAR(6, *j_data.position[16], EPS);
    EXPECT_NEAR(6, *j_data.position[17], EPS);
    EXPECT_NEAR(6, *j_data.position[18], EPS);

  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}