#pragma once

// automation state

const int HumanTakeOver = 1;
const int AutomatedSystem = 2;

// AT constants
const int noTransition = 1;
const int DIDC = 2;
const int AIDC = 3;

const double ODD_MAX_BRAKE = -4; // the maximum deceleration of BADF, when the

// Lane change
const int	changeLeft = 1;
const int	changeRight = -1;
const int	noChangeLane = 0;

// simulation scenario
const int  motorway = 1;
const int  urban = 2;

// vehicle types
const int BADF = 700;
const int EADF = 800;

