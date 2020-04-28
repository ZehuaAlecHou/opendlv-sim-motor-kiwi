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

#include <cmath>
#include <iostream>

#include "single-track-model.hpp"

SingleTrackModel::SingleTrackModel() noexcept:
  m_leftWheelSpeedMutex{},
  m_rightWheelSpeedMutex{},
  m_longitudinalSpeed{0.0f},
  m_lateralSpeed{0.0f},
  m_yawRate{0.0f},
  m_leftWheelSpeed{0.0f},
  m_rightWheelSpeed{0.0f}
{
}

void SingleTrackModel::setLeftWheelSpeed(opendlv::proxy::LeftWheelSpeedRequest const &leftWheelSpeed) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftWheelSpeedMutex);
  m_leftWheelSpeed = leftWheelSpeed.speed();
}

void SingleTrackModel::setRightWheelSpeed(opendlv::proxy::RightWheelSpeedRequest const &rightWheelSpeed) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightWheelSpeedMutex);
  m_rightWheelSpeed = rightWheelSpeed.speed();
}

opendlv::sim::KinematicState SingleTrackModel::step(double dt) noexcept
{ 
  
  double const pedalSpeedGain{0.5};

  double const mass{1.0};
  double const momentOfInertiaZ{0.1};
  double const length{0.22};
  double const frontToCog{0.11};
  double const rearToCog{length - frontToCog};
  double const corneringStiffnessFront{1.0};
  double const corneringStiffnessRear{1.0};
  
  float leftWheelSpeedCopy;
  float rightWheelSpeedCopy;
  {
    std::lock_guard<std::mutex> lock1(m_leftWheelSpeedMutex);
    std::lock_guard<std::mutex> lock2(m_rightWheelSpeedMutex);
    leftWheelSpeedCopy = m_groundSteeringAngle;
    rightWheelSpeedCopy = m_pedalPosition;
  }
  double const radius = 0.12;
  double const yawRateDot = (rightWheelSpeedCopy-leftWheelSpeedCopy)/2/radius;
  m_yawRate += yawRateDot * dt;
  double const speed = (rightWheelSpeedCopy+leftWheelSpeedCopy)/2;
  double const longitudinalSpeedDot = speed*cos(m_yawRate);
  double const lateralSpeedDot = speed*sin(m_yawRate);
  m_longitudinalSpeed += longitudinalSpeedDot * dt;
  m_lateralSpeed += lateralSpeedDot * dt;

  opendlv::sim::KinematicState kinematicState;
  kinematicState.vx(static_cast<float>(m_longitudinalSpeed));
  kinematicState.vy(static_cast<float>(m_lateralSpeed));
  kinematicState.yawRate(static_cast<float>(m_yawRate));

  return kinematicState;
}
