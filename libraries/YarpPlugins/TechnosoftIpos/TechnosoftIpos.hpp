// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TECHNOSOFT_IPOS_HPP__
#define __TECHNOSOFT_IPOS_HPP__

#include "TechnosoftIposBase.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup TechnosoftIpos
 * @brief Contains roboticslab::TechnosoftIpos.
 */

/**
 * @ingroup TechnosoftIpos
 * @brief Implementation for the Technosoft iPOS as a single CAN bus joint (controlboard raw interfaces).
 */
class TechnosoftIpos : public yarp::dev::DeviceDriver,
                       public yarp::dev::IAxisInfoRaw,
                       public yarp::dev::IControlLimitsRaw,
                       public yarp::dev::IControlModeRaw,
                       public yarp::dev::ICurrentControlRaw,
                       public yarp::dev::IEncodersTimedRaw,
                       public yarp::dev::IJointFaultRaw,
                       public yarp::dev::IMotorRaw,
                       public yarp::dev::IMotorEncodersRaw,
                       public yarp::dev::IPositionControlRaw,
                       public yarp::dev::IPositionDirectRaw,
                       public yarp::dev::IRemoteVariablesRaw,
                       public yarp::dev::ITorqueControlRaw,
                       public yarp::dev::IVelocityControlRaw,
                       public ICanBusSharer
{
public:

    ~TechnosoftIpos() override
    { close(); }

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    unsigned int getId() override
    { return impl->getId(); }

    std::vector<unsigned int> getAdditionalIds() override
    { return impl->getAdditionalIds(); }

    bool notifyMessage(const can_message & message) override
    { return impl->notifyMessage(message); }

    bool initialize() override
    { return impl->initialize(); }

    bool finalize() override
    { return impl->finalize(); }

    bool registerSender(CanSenderDelegate * sender) override
    { return impl->registerSender(sender); }

    bool synchronize() override
    { return impl->synchronize(); }

    //  --------- IAxisInfoRaw declarations. Implementation in IAxisInfoRawImpl.cpp ---------

    bool getAxisNameRaw(int axis, std::string & name) override
    { return impl->getAxisNameRaw(axis, name); }

    bool getJointTypeRaw(int axis, yarp::dev::JointTypeEnum & type) override
    { return impl->getJointTypeRaw(axis, type); }

    //  --------- IControlLimitsRaw declarations. Implementation in IControlLimitsRawImpl.cpp ---------

    bool setLimitsRaw(int axis, double min, double max) override
    { return impl->setLimitsRaw(axis, min, max); }

    bool getLimitsRaw(int axis, double * min, double * max) override
    { return impl->getLimitsRaw(axis, min, max); }

    bool setVelLimitsRaw(int axis, double min, double max) override
    { return impl->setVelLimitsRaw(axis, min, max); }

    bool getVelLimitsRaw(int axis, double * min, double * max) override
    { return impl->getVelLimitsRaw(axis, min, max); }

    //  --------- IControlModeRaw declarations. Implementation in IControlModeRawImpl.cpp ---------

    bool getControlModeRaw(int j, int * mode) override
    { return impl->getControlModeRaw(j, mode); }

    bool getControlModesRaw(int * modes) override
    { return impl->getControlModesRaw(modes); }

    bool getControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return impl->getControlModesRaw(n_joint, joints, modes); }

    bool setControlModeRaw(int j, int mode) override
    { return impl->setControlModeRaw(j, mode); }

    bool setControlModesRaw(int * modes) override
    { return impl->setControlModesRaw(modes); }

    bool setControlModesRaw(int n_joint, const int * joints, int * modes) override
    { return impl->setControlModesRaw(n_joint, joints, modes); }

    //  --------- ICurrentControlRaw declarations. Implementation in ICurrentControlRawImpl.cpp ---------

    //bool getNumberOfMotorsRaw(int * number) override;

    bool getCurrentRaw(int m, double * curr) override
    { return impl->getCurrentRaw(m, curr); }

    bool getCurrentsRaw(double * currs) override
    { return impl->getCurrentsRaw(currs); }

    bool getCurrentRangeRaw(int m, double * min, double * max) override
    { return impl->getCurrentRangeRaw(m, min, max); }

    bool getCurrentRangesRaw(double * min, double * max) override
    { return impl->getCurrentRangesRaw(min, max); }

    bool setRefCurrentRaw(int m, double curr) override
    { return impl->setRefCurrentRaw(m, curr); }

    bool setRefCurrentsRaw(const double * currs) override
    { return impl->setRefCurrentsRaw(currs); }

    bool setRefCurrentsRaw(int n_motor, const int * motors, const double * currs) override
    { return impl->setRefCurrentsRaw(n_motor, motors, currs); }

    bool getRefCurrentRaw(int m, double * curr) override
    { return impl->getRefCurrentRaw(m, curr); }

    bool getRefCurrentsRaw(double * currs) override
    { return impl->getRefCurrentsRaw(currs); }

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getAxes(int * ax) override
    { return impl->getAxes(ax); }

    bool resetEncoderRaw(int j) override
    { return impl->resetEncoderRaw(j); }

    bool resetEncodersRaw() override
    { return impl->resetEncodersRaw(); }

    bool setEncoderRaw(int j, double val) override
    { return impl->setEncoderRaw(j, val); }

    bool setEncodersRaw(const double * vals) override
    { return impl->setEncodersRaw(vals); }

    bool getEncoderRaw(int j, double * v) override
    { return impl->getEncoderRaw(j, v); }

    bool getEncodersRaw(double * encs) override
    { return impl->getEncodersRaw(encs); }

    bool getEncoderSpeedRaw(int j, double * sp) override
    { return impl->getEncoderSpeedRaw(j, sp); }

    bool getEncoderSpeedsRaw(double * spds) override
    { return impl->getEncoderSpeedsRaw(spds); }

    bool getEncoderAccelerationRaw(int j, double * spds) override
    { return impl->getEncoderAccelerationRaw(j, spds); }

    bool getEncoderAccelerationsRaw(double * accs) override
    { return impl->getEncoderAccelerationsRaw(accs); }

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getEncoderTimedRaw(int j, double * encs, double * time) override
    { return impl->getEncoderTimedRaw(j, encs, time); }

    bool getEncodersTimedRaw(double * encs, double * time) override
    { return impl->getEncodersTimedRaw(encs, time); }

    //  ---------- IJointFaultRaw declarations. Implementation in IJointFaultRawImpl.cpp ----------

    bool getLastJointFaultRaw(int j, int & fault, std::string & message) override
    { return impl->getLastJointFaultRaw(j, fault, message); }

    //  --------- IMotorRaw declarations. Implementation in IMotorRawImpl.cpp ---------

    bool getNumberOfMotorsRaw(int * num) override
    { return impl->getNumberOfMotorsRaw(num); }

    bool getTemperatureRaw(int m, double * val) override
    { return impl->getTemperatureRaw(m, val); }

    bool getTemperaturesRaw(double * vals) override
    { return impl->getTemperaturesRaw(vals); }

    bool getTemperatureLimitRaw(int m, double * temp) override
    { return impl->getTemperatureLimitRaw(m, temp); }

    bool setTemperatureLimitRaw(int m, double temp) override
    { return impl->setTemperatureLimitRaw(m, temp); }

    bool getGearboxRatioRaw(int m, double * val) override
    { return impl->getGearboxRatioRaw(m, val); }

    bool setGearboxRatioRaw(int m, double val) override
    { return impl->setGearboxRatioRaw(m, val); }

    //  --------- IMotorEncodersRaw declarations. Implementation in IMotorEncodersRawImpl.cpp ---------

    bool getNumberOfMotorEncodersRaw(int * num) override
    { return impl->getNumberOfMotorEncodersRaw(num); }

    bool resetMotorEncoderRaw(int m) override
    { return impl->resetMotorEncoderRaw(m); }

    bool resetMotorEncodersRaw() override
    { return impl->resetMotorEncodersRaw(); }

    bool setMotorEncoderCountsPerRevolutionRaw(int m, double cpr) override
    { return impl->setMotorEncoderCountsPerRevolutionRaw(m, cpr); }

    bool getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr) override
    { return impl->getMotorEncoderCountsPerRevolutionRaw(m, cpr); }

    bool setMotorEncoderRaw(int m, double val) override
    { return impl->setMotorEncoderRaw(m, val); }

    bool setMotorEncodersRaw(const double * vals) override
    { return impl->setMotorEncodersRaw(vals); }

    bool getMotorEncoderRaw(int m, double * v) override
    { return impl->getMotorEncoderRaw(m, v); }

    bool getMotorEncodersRaw(double * encs) override
    { return impl->getMotorEncodersRaw(encs); }

    bool getMotorEncoderTimedRaw(int m, double * encs, double * stamp) override
    { return impl->getMotorEncoderTimedRaw(m, encs, stamp); }

    bool getMotorEncodersTimedRaw(double * encs, double * stamps) override
    { return impl->getMotorEncodersTimedRaw(encs, stamps); }

    bool getMotorEncoderSpeedRaw(int m, double * sp) override
    { return impl->getMotorEncoderSpeedRaw(m, sp); }

    bool getMotorEncoderSpeedsRaw(double * spds) override
    { return impl->getMotorEncoderSpeedsRaw(spds); }

    bool getMotorEncoderAccelerationRaw(int m, double * spds) override
    { return impl->getMotorEncoderAccelerationRaw(m, spds); }

    bool getMotorEncoderAccelerationsRaw(double * vaccs) override
    { return impl->getMotorEncoderAccelerationsRaw(vaccs); }

    // ------- IPositionControlRaw declarations. Implementation in IPositionControlRawImpl.cpp -------

    //bool getAxes(int * ax) override;

    bool positionMoveRaw(int j, double ref) override
    { return impl->positionMoveRaw(j, ref); }

    bool positionMoveRaw(const double * refs) override
    { return impl->positionMoveRaw(refs); }

    bool positionMoveRaw(int n_joint, const int * joints, const double * refs) override
    { return impl->positionMoveRaw(n_joint, joints, refs); }

    bool relativeMoveRaw(int j, double delta) override
    { return impl->relativeMoveRaw(j, delta); }

    bool relativeMoveRaw(const double * deltas) override
    { return impl->relativeMoveRaw(deltas); }

    bool relativeMoveRaw(int n_joint, const int * joints, const double * deltas) override
    { return impl->relativeMoveRaw(n_joint, joints, deltas); }

    bool checkMotionDoneRaw(int j, bool * flag) override
    { return impl->checkMotionDoneRaw(j, flag); }

    bool checkMotionDoneRaw(bool * flag) override
    { return impl->checkMotionDoneRaw(flag); }

    bool checkMotionDoneRaw(int n_joint, const int * joints, bool * flag) override
    { return impl->checkMotionDoneRaw(n_joint, joints, flag); }

    bool setRefSpeedRaw(int j, double sp) override
    { return impl->setRefSpeedRaw(j, sp); }

    bool setRefSpeedsRaw(const double * spds) override
    { return impl->setRefSpeedsRaw(spds); }

    bool setRefSpeedsRaw(int n_joint, const int * joints, const double * spds) override
    { return impl->setRefSpeedsRaw(n_joint, joints, spds); }

    bool setRefAccelerationRaw(int j, double acc) override
    { return impl->IPositionControlRaw::setRefAccelerationRaw(j, acc); }

    bool setRefAccelerationsRaw(const double * accs) override
    { return impl->IPositionControlRaw::setRefAccelerationsRaw(accs); }

    bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override
    { return impl->IPositionControlRaw::setRefAccelerationsRaw(n_joint, joints, accs); }

    bool getRefSpeedRaw(int j, double * ref) override
    { return impl->getRefSpeedRaw(j, ref); }

    bool getRefSpeedsRaw(double * spds) override
    { return impl->getRefSpeedsRaw(spds); }

    bool getRefSpeedsRaw(int n_joint, const int * joints, double * spds) override
    { return impl->getRefSpeedsRaw(n_joint, joints, spds); }

    bool getRefAccelerationRaw(int j, double * acc) override
    { return impl->IPositionControlRaw::getRefAccelerationRaw(j, acc); }

    bool getRefAccelerationsRaw(double * accs) override
    { return impl->IPositionControlRaw::getRefAccelerationsRaw(accs); }

    bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override
    { return impl->IPositionControlRaw::getRefAccelerationsRaw(n_joint, joints, accs); }

    bool stopRaw(int j) override
    { return impl->IPositionControlRaw::stopRaw(j); }

    bool stopRaw() override
    { return impl->IPositionControlRaw::stopRaw(); }

    bool stopRaw(int n_joint, const int * joints) override
    { return impl->IPositionControlRaw::stopRaw(n_joint, joints); }

    bool getTargetPositionRaw(int joint, double * ref) override
    { return impl->getTargetPositionRaw(joint, ref); }

    bool getTargetPositionsRaw(double * refs) override
    { return impl->getTargetPositionsRaw(refs); }

    bool getTargetPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return impl->getTargetPositionsRaw(n_joint, joints, refs); }

    // ------- IPositionDirectRaw declarations. Implementation in IPositionDirectRawImpl.cpp -------

    //bool getAxes(int * ax) override;

    bool setPositionRaw(int j, double ref) override
    { return impl->setPositionRaw(j, ref); }

    bool setPositionsRaw(const double * refs) override
    { return impl->setPositionsRaw(refs); }

    bool setPositionsRaw(int n_joint, const int * joints, const double * refs) override
    { return impl->setPositionsRaw(n_joint, joints, refs); }

    bool getRefPositionRaw(int joint, double * ref) override
    { return impl->getRefPositionRaw(joint, ref); }

    bool getRefPositionsRaw(double * refs) override
    { return impl->getRefPositionsRaw(refs); }

    bool getRefPositionsRaw(int n_joint, const int * joints, double * refs) override
    { return impl->getRefPositionsRaw(n_joint, joints, refs); }

    // ------- IRemoteVariablesRaw declarations. Implementation in IRemoteVariablesRawImpl.cpp -------

    bool getRemoteVariableRaw(std::string key, yarp::os::Bottle & val) override
    { return impl->getRemoteVariableRaw(key, val); }

    bool setRemoteVariableRaw(std::string key, const yarp::os::Bottle & val) override
    { return impl->setRemoteVariableRaw(key, val); }

    bool getRemoteVariablesListRaw(yarp::os::Bottle * listOfKeys) override
    { return impl->getRemoteVariablesListRaw(listOfKeys); }

    // -------- ITorqueControlRaw declarations. Implementation in ITorqueControlRawImpl.cpp --------

    //bool getAxes(int * ax) override;

    bool getRefTorqueRaw(int j, double * t) override
    { return impl->getRefTorqueRaw(j, t); }

    bool getRefTorquesRaw(double * t) override
    { return impl->getRefTorquesRaw(t); }

    bool setRefTorqueRaw(int j, double t) override
    { return impl->setRefTorqueRaw(j, t); }

    bool setRefTorquesRaw(const double * t) override
    { return impl->setRefTorquesRaw(t); }

    bool getTorqueRaw(int j, double * t) override
    { return impl->getTorqueRaw(j, t); }

    bool getTorquesRaw(double * t) override
    { return impl->getTorquesRaw(t); }

    bool getTorqueRangeRaw(int j, double * min, double * max) override
    { return impl->getTorqueRangeRaw(j, min, max); }

    bool getTorqueRangesRaw(double * min, double * max) override
    { return impl->getTorqueRangesRaw(min, max); }

    bool getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params) override
    { return impl->getMotorTorqueParamsRaw(j, params); }

    bool setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params) override
    { return impl->setMotorTorqueParamsRaw(j, params); }

    //  --------- IVelocityControlRaw declarations. Implementation in IVelocityControlRawImpl.cpp ---------

    //bool getAxes(int * ax) override;

    bool velocityMoveRaw(int j, double sp) override
    { return impl->velocityMoveRaw(j, sp); }

    bool velocityMoveRaw(const double * sp) override
    { return impl->velocityMoveRaw(sp); }

    bool velocityMoveRaw(int n_joint, const int * joints, const double * spds) override
    { return impl->velocityMoveRaw(n_joint, joints, spds); }

    bool getRefVelocityRaw(int joint, double * vel) override
    { return impl->getRefVelocityRaw(joint, vel); }

    bool getRefVelocitiesRaw(double * vels) override
    { return impl->getRefVelocitiesRaw(vels); }

    bool getRefVelocitiesRaw(int n_joint, const int * joints, double * vels) override
    { return impl->getRefVelocitiesRaw(n_joint, joints, vels); }

    //bool setRefAccelerationRaw(int j, double acc) override;
    //bool setRefAccelerationsRaw(const double * accs) override;
    //bool setRefAccelerationsRaw(int n_joint, const int * joints, const double * accs) override;
    //bool getRefAccelerationRaw(int j, double * acc) override;
    //bool getRefAccelerationsRaw(double * accs) override;
    //bool getRefAccelerationsRaw(int n_joint, const int * joints, double * accs) override;
    //bool stopRaw(int j) override;
    //bool stopRaw() override;
    //bool stopRaw(int n_joint, const int *joints) override;

private:

    TechnosoftIposBase * impl {nullptr};
};

} // namespace roboticslab

#endif // __TECHNOSOFT_IPOS_HPP__
