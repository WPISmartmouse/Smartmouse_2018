#include <tuple>
#include <random>
#include <gazebo/gazebo_core.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <common/Direction.h>
#include <common/AbstractMaze.h>

namespace gazebo {

  static constexpr double X_OFFSET = -(AbstractMaze::MAZE_SIZE * AbstractMaze::UNIT_DIST / 2); // meters
  static constexpr double Y_OFFSET= AbstractMaze::MAZE_SIZE * AbstractMaze::UNIT_DIST / 2; // meters

  class MazeFactory : public WorldPlugin {

  public:
    MazeFactory();

    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    void Regenerate(ConstGzStringPtr &msg);

  private:

    char to_char(Direction dir);

    /// \brief load from maze_base/model.sdf
    sdf::ElementPtr LoadModel();

    /// \brief insert allll the walls
    // \param  base_link this should be the base link of the maze
    void InsertWallsFromFile(sdf::ElementPtr base_link);

    void RemoveWalls();

    /// \brief insert a wall collision and visual into the given link
    // \param link the link you're inserting the models to
    void InsertWall(sdf::ElementPtr link, int row,
                    int col, Direction dir);

    std::list<sdf::ElementPtr> CreateWallVisual(int row,
                                                int col, Direction dir);

    sdf::ElementPtr CreateWallCollision(int row,
                                        int col, Direction dir);

    msgs::Geometry *CreateBoxGeometry(double x, double y, double z);

    msgs::Geometry *CreateCylinderGeometry(double r, double h);

    msgs::Pose *CreatePose(int row, int col, double z, Direction dir);

    std::list<sdf::ElementPtr> all_wall_elements;

    transport::NodePtr node;
    transport::SubscriberPtr regen_sub;

    physics::WorldPtr parent;

    sdf::SDFPtr modelSDF;

    std::string maze_filename;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> neighbor_dist;
    std::uniform_int_distribution<int> remove_dist;

    const static double WALL_LENGTH,
            WALL_HEIGHT,
            WALL_THICKNESS,
            RED_HIGHLIGHT_THICKNESS,
            INDICATOR_RADIUS,
            UNIT,
            BASE_HEIGHT;

    bool visited[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE];

    /// \brief matrix of bool arrays of size four.
    // [0][0][0] represents the connection of 0,0 to the north
    // [2][1][3] represents the connection of 2,1 to the west
    // ect...
    bool connected[AbstractMaze::MAZE_SIZE][AbstractMaze::MAZE_SIZE][4];//order N, E, S, W

  };
}

