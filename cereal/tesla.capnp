using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.comma.openpilot.cereal");
$Java.outerClassname("Alca");

@0xca61a35dedbd6327;

const interfaceVersion :Float32 = 4.0;

struct ALCAStatus { 
  # ALCA info
  alcaEnabled @0 :Bool;
  alcaDirection @1 :Int8;
  alcaTotalSteps @2 :UInt16;
  alcaError @3 :Bool;
}

struct ALCAState {
  alcaError @0 :Bool;
  alcaCancelling @1 :Bool;
  alcaEnabled @2 :Bool;
  alcaLaneWidth @3 :Float32;
  alcaStep @4 :UInt8;
  alcaTotalSteps @5 :UInt16;
  alcaDirection @6 :Int8;
}

struct TeslaRadarPoint {
  trackId @0 :UInt64;  # no trackId reuse
  # some TeslaBosch specific items
  objectClass @1 :UInt8; # 0-unknown 1-four wheel vehicle 2-two wheel vehicle 3-pedestrian 4-construction element
  dz @2 :Float32; # height in meter
  movingState @3 :UInt8; # 0-indeterminate 1-moving 2-stopped 3-standing
  length @4 :Float32; # length in meters
  obstacleProb @5 :Float32; # probability to be an obstacle
  timeStamp @6 :UInt64; #timestamp when the pair was received
}

struct ICCarsLR {
    v1Type @0 :Int8;
    v1Dx @1 :Float32;
    v1Vrel @2 :Float32;
    v1Dy @3 :Float32;
    v1Id @4 :Int8;
    v2Type @5 :Int8;
    v2Dx @6 :Float32;
    v2Vrel @7 :Float32;
    v2Dy @8 :Float32;
    v2Id @9 :Int8;
    v3Type @10 :Int8;
    v3Dx @11 :Float32;
    v3Vrel @12 :Float32;
    v3Dy @13 :Float32;
    v3Id @14 :Int8;
    v4Type @15 :Int8;
    v4Dx @16 :Float32;
    v4Vrel @17 :Float32;
    v4Dy @18 :Float32;
    v4Id @19 :Int8;
}

struct ICLeads {
  lead1trackId @0 :Int8;
  lead1oClass @1 :Int8;
  lead1length @2 :Float32;
  lead2trackId @3 :Int8;
  lead2oClass @4 :Int8;
  lead2length @5 :Float32;
}

struct AHBinfo {
  source @0 :UInt8; #use 0 for radar, 1 for camera, 2 for other
  radarCarDetected @1 :Bool;
  cameraCarDetected @2 :Bool;
}

struct TeslaLeadPoint {
  trackId @0 :UInt8;  # no trackId reuse
  oClass @1 :UInt8; # 0-unknown 1-four wheel vehicle 2-two wheel vehicle 3-pedestrian 4-construction element
  length @2 :Float32; # length in meters
}

