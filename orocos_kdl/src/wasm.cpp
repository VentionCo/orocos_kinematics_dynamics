#include "chain.hpp"
#include "solveri.hpp"
#include "chainfksolver.hpp"
#include "chainfksolverpos_recursive.hpp"
#include "chainiksolver.hpp"
#include "chainiksolvervel_pinv.hpp"
#include "chainiksolver.hpp"
#include "chainiksolverpos_lma.hpp"
#include "chainiksolverpos_nr.hpp"
#include "frames.hpp"
#include <Eigen/Core>
#include <vector>

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

KDL::Frame Frame_mul(const KDL::Frame& lhs,const KDL::Frame& rhs)
{
  return KDL::Frame(lhs.M*rhs.M,lhs.M*rhs.p+lhs.p);
};

EMSCRIPTEN_BINDINGS (c) {
  class_<KDL::Chain>("Chain")
    .constructor()
    .function("addChain", &KDL::Chain::addChain)
    .function("addSegment", &KDL::Chain::addSegment)
    ;

  class_<KDL::Segment>("Segment")
    .constructor<KDL::Joint>()
    .constructor<KDL::Joint, KDL::Frame>()
    ;

  class_<KDL::Joint>("Joint")
    .constructor<KDL::Joint::JointType>()
    .constructor<KDL::Vector, KDL::Vector, KDL::Joint::JointType>()
    .function("getNrOfJoints", &KDL::Chain::getNrOfJoints)
    ;

  class_<KDL::Frame>("Frame")
    .constructor<KDL::Rotation, KDL::Vector>()
    .class_function("DH", &KDL::Frame::DH)
    ;

  class_<KDL::Rotation>("Rotation")
    .class_function("Identity", &KDL::Rotation::Identity)
    .class_function("EulerZYX", &KDL::Rotation::EulerZYX)
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
    .constructor<KDL::Chain>()
    ;

  class_<KDL::ChainIkSolverVel, base<KDL::SolverI>>("ChainIkSolverVel");
  class_<KDL::ChainIkSolverVel_pinv, base<KDL::ChainIkSolverVel>>("ChainIkSolverVel_pinv")
    .constructor<KDL::Chain>()
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

  function("getStdVectorFromJntArray", &getStdVectorFromJntArray);

  // replaces: Frame operator *(const Frame& lhs,const Frame& rhs)
  function("Frame_mul", &Frame_mul);
}