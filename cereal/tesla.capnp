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


