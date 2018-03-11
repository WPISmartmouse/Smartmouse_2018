# allow us to import from existing ignition msgs
file(GLOB PROTOS *.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER ${PROTOS})

# Variables needed to propagate through modules
# If more than one layer of cmake use CACHE INTERNAL ...
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)

foreach (PROTO_H ${PROTO_HEADER})
  set_property(SOURCE ${PROTO_H} PROPERTY SKIP_AUTOGEN ON)
endforeach ()

foreach (PROTO_CPP ${PROTO_SRC})
  set_property(SOURCE ${PROTO_CPP} PROPERTY SKIP_AUTOGEN ON)
endforeach ()

add_library(msgs msgs.cpp ${PROTO_SRC} ${PROTO_HEADER})
target_link_libraries(msgs sim_core_common)
