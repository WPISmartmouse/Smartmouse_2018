# allow us to import from existing ignition msgs
set(PROTOBUF_IMPORT_DIRS /home/peter/Projects/ign/ign-msgs)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER
        gui_actions.proto
        maze_location.proto
        robot_command.proto
        state.proto
        )

# Variables needed to propagate through modules
# If more than one layer of cmake use CACHE INTERNAL ...
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIRS} PARENT_SCOPE)
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES} PARENT_SCOPE)
set(PROTO_SRC ${PROTO_SRC} PARENT_SCOPE)
set(PROTO_HEADER ${PROTO_HEADER} PARENT_SCOPE)

add_library(msgs STATIC ${PROTO_SRC} ${PROTO_HEADER})
