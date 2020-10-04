
"use strict";

let MotorStatus = require('./MotorStatus.js');
let ControllerState = require('./ControllerState.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let YawrateCommand = require('./YawrateCommand.js');
let MotorCommand = require('./MotorCommand.js');
let RawMagnetic = require('./RawMagnetic.js');
let Supply = require('./Supply.js');
let RC = require('./RC.js');
let HeightCommand = require('./HeightCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let RawRC = require('./RawRC.js');
let Altitude = require('./Altitude.js');
let RuddersCommand = require('./RuddersCommand.js');
let RawImu = require('./RawImu.js');
let MotorPWM = require('./MotorPWM.js');
let HeadingCommand = require('./HeadingCommand.js');
let Altimeter = require('./Altimeter.js');
let ServoCommand = require('./ServoCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let Compass = require('./Compass.js');

module.exports = {
  MotorStatus: MotorStatus,
  ControllerState: ControllerState,
  VelocityZCommand: VelocityZCommand,
  YawrateCommand: YawrateCommand,
  MotorCommand: MotorCommand,
  RawMagnetic: RawMagnetic,
  Supply: Supply,
  RC: RC,
  HeightCommand: HeightCommand,
  VelocityXYCommand: VelocityXYCommand,
  RawRC: RawRC,
  Altitude: Altitude,
  RuddersCommand: RuddersCommand,
  RawImu: RawImu,
  MotorPWM: MotorPWM,
  HeadingCommand: HeadingCommand,
  Altimeter: Altimeter,
  ServoCommand: ServoCommand,
  PositionXYCommand: PositionXYCommand,
  AttitudeCommand: AttitudeCommand,
  ThrustCommand: ThrustCommand,
  Compass: Compass,
};
