#include <Eigen/Core>
#include <vector>

#include "chain.hpp"
#include "frames.hpp"
#include "solveri.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolver.hpp"
#include "chainiksolverpos_lma.hpp"

#include "emscripten/bind.h"

using namespace emscripten;

const std::vector<double> getStdVectorFromJntArray(const KDL::JntArray& array)
{
  std::vector<double> v;

  for (int i = 0; i < array.data.rows(); i++) {
    v.push_back(array.data[i]);
  }

  return v;
};

const std::vector<double> getStdVectorFromVector(const KDL::Vector& vector)
{
  std::vector<double> v;

  for (int i = 0; i < sizeof(vector.data) / sizeof(vector.data[0]); i++) {
    v.push_back(vector.data[i]);
  }

  return v;
};

const std::vector<double> getStdVectorFromRotation(const KDL::Rotation& rotation)
{
  std::vector<double> v;

  for (int i = 0; i < sizeof(rotation.data) / sizeof(rotation.data[0]); i++) {
    v.push_back(rotation.data[i]);
  }

  return v;
};

KDL::Frame Frame_mul(const KDL::Frame& lhs,const KDL::Frame& rhs)
{
  return KDL::Frame(lhs.M*rhs.M,lhs.M*rhs.p+lhs.p);
};

KDL::Vector Frame_getPositionVector(const KDL::Frame& frame)
{
  return frame.p;
}

KDL::Rotation Frame_getRotation(const KDL::Frame& frame)
{
  return frame.M;
}

void setJntArrayDataAtIndex(KDL::JntArray &array, unsigned int i, double jnt_value)
{
  array.data[i] = jnt_value;
};

EMSCRIPTEN_BINDINGS (c) {
  class_<KDL::Chain>("Chain")
    .constructor()
    .function("addChain", &KDL::Chain::addChain)
    .function("addSegment", &KDL::Chain::addSegment)
    .function("getSegment", &KDL::Chain::getSegment)
    ;

  class_<KDL::Segment>("Segment")
    .constructor<KDL::Joint>()
    .constructor<KDL::Joint, KDL::Frame>()
    .function("pose", &KDL::Segment::pose)
    .function("getJoint", &KDL::Segment::getJoint)
    .function("getFrameToTip", &KDL::Segment::getFrameToTip)
    ;

  class_<KDL::Joint>("Joint")
    .constructor<KDL::Joint::JointType>()
    .constructor<KDL::Vector, KDL::Vector, KDL::Joint::JointType>()
    .function("getNrOfJoints", &KDL::Chain::getNrOfJoints)
    .function("getNrOfSegments", &KDL::Chain::getNrOfSegments)
    ;

  class_<KDL::Frame>("Frame")
    .constructor<KDL::Rotation, KDL::Vector>()
    .class_function("DH", &KDL::Frame::DH)
    .function("Make4x4", &KDL::Frame::Make4x4, allow_raw_pointers())
    ;

  class_<KDL::Rotation>("Rotation")
    .constructor<KDL::Vector, KDL::Vector, KDL::Vector>()
    .class_function("Identity", &KDL::Rotation::Identity)
    .class_function("EulerZYX", &KDL::Rotation::EulerZYX)
    .class_function("RotX", &KDL::Rotation::RotX)
    .class_function("RotY", &KDL::Rotation::RotY)
    .class_function("RotZ", &KDL::Rotation::RotZ)
    .function("GetRot", &KDL::Rotation::GetRot)
    ;

  class_<KDL::Vector>("Vector")
    .constructor<double, double, double>()
    .class_function("Zero", &KDL::Vector::Zero)
    ;

  enum_<KDL::Joint::JointType>("JointType")
    .value("None", KDL::Joint::None)
    .value("RotX", KDL::Joint::RotX)
    .value("RotY", KDL::Joint::RotY)
    .value("RotZ", KDL::Joint::RotZ)
    ;

  class_<KDL::SolverI>("SolverI");

  class_<KDL::ChainFkSolverPos, base<KDL::SolverI>>("ChainFkSolverPos");
  class_<KDL::ChainFkSolverPos_recursive, base<KDL::ChainFkSolverPos>>("ChainFkSolverPos_recursive")
    .constructor<KDL::Chain&>()
    .function("JntToCart", &KDL::ChainFkSolverPos_recursive::JntToCart)
    ;

  class_<KDL::ChainIkSolverPos, base<KDL::SolverI>>("ChainIkSolverPos");
  class_<KDL::ChainIkSolverPos_LMA, base<KDL::ChainIkSolverPos>>("ChainIkSolverPos_LMA")
    .constructor<KDL::Chain&>()
    .function("CartToJnt", &KDL::ChainIkSolverPos_LMA::CartToJnt)
    ;

  register_vector<double>("vector<double>");
  class_<KDL::JntArray>("JntArray")
    .constructor<unsigned int>()
    .property("data", &KDL::JntArray::data)
    ;

  function("getStdVectorFromVector", &getStdVectorFromVector);
  function("getStdVectorFromJntArray", &getStdVectorFromJntArray);
  function("getStdVectorFromRotation", &getStdVectorFromRotation);
  function("setJntArrayDataAtIndex", &setJntArrayDataAtIndex);

  // replaces: Frame operator *(const Frame& lhs,const Frame& rhs)
  function("Frame_mul", &Frame_mul);
  function("Frame_getPositionVector", &Frame_getPositionVector);
  function("Frame_getRotation", &Frame_getRotation);
}