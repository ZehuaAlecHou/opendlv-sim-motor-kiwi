/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "differentially-steered-model.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq") || 0 == commandlineArguments.count("frame-id")) {
    std::cerr << argv[0] << " is a dynamics model for the Chalmers Kiwi platform." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --frame-id=<ID of frame (used for integration)> --freq=<Model frequency> --cid=<OpenDaVINCI session>" << std::endl;
    std::cerr << "Example: " << argv[0] << " --frame-id=0 --freq=100 --cid=111" << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    uint32_t const FRAME_ID = std::stoi(commandlineArguments["frame-id"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;
    
    DifferentiallySteeredModel differentiallySteeredModel;

    auto onWheelSpeedRequest{[&differentiallySteeredModel](cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
          auto leftWheelSpeedRequest = cluon::extractMessage<opendlv::proxy::LeftWheelSpeedRequest>(std::move(envelope));
          differentiallySteeredModel.setLeftWheelSpeed(leftWheelSpeedRequest);
         }
        else if (senderStamp == 1) {
          auto rightWheelSpeedRequest = cluon::extractMessage<opendlv::proxy::RightWheelSpeedRequest>(std::move(envelope));
          differentiallySteeredModel.setRightWheelSpeed(rightWheelSpeedRequest);
         }
      }};
   
    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::WheelSpeedRequest::ID(), onWheelSpeedRequest);
    
    auto atFrequency{[&FRAME_ID, &VERBOSE, &DT, &differentiallySteeredModel, &od4]() -> bool
      {
        opendlv::sim::KinematicState kinematicState = differentiallySteeredModel.step(DT);

        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(kinematicState, sampleTime, FRAME_ID);
        if (VERBOSE) {
          std::cout << "Kinematic state with id " << FRAME_ID
            << " is at velocity [vx=" << kinematicState.vx() << ", vy=" << kinematicState.vy() << ", vz="
            << kinematicState.vz() << "] with the rotation rate [rollRate=" << kinematicState.rollRate() << ", pitchRate="
            << kinematicState.pitchRate() << ", yawRate=" << kinematicState.yawRate() << "]." << std::endl;
        }

        return true;
      }};

    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
