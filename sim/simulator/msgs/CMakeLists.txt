# allow us to import from existing ignition msgs
file(GLOB PROTOS *.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER ${PROTOS})

# Variables needed to propagate through modules
# If more than one layer of cmake use CACHE INTERNAL ...
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)

add_library(msgs STATIC ${PROTO_SRC} ${PROTO_HEADER})
