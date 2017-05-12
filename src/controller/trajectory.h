#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Core>
#include <json/json.h>
#include <fstream>
#include <exception>
#include <vector>

#include <iostream>

class ToolPath {
public:
  int tool;
  std::vector<Eigen::Vector2d> points;
};

class TrajectoryParsingException : public std::exception {
public:
  TrajectoryParsingException(const std::string &message) :
  message(message)
  {}

  virtual const char* what() const throw()
  {
    return message.c_str();
  }

private:
  const std::string message;
};

typedef std::vector<ToolPath>::iterator TrajectorySequenceIterator;
typedef std::vector<Eigen::Vector2d>::iterator TrajectoryToolPathPointIterator;

class Trajectory {

public:
  static Trajectory fromFile(const std::string &file_name) {
    Trajectory t;
    t.parseFile(file_name);
    return t;
  }

  std::vector<ToolPath> sequence;

private:
  Trajectory()
  {
    // Make constructor private.
  }

  ToolPath parseToolPath(Json::Value tool_path_node) {
    ToolPath result;
    Json::Value tool_node = tool_path_node["tool"];
    if (!tool_node.isInt()) {
      throw TrajectoryParsingException("No tool or invalid tool specified");
    }
    result.tool = tool_node.asInt();
    Json::Value points = tool_path_node["points"];
    if (!points) {
      throw TrajectoryParsingException("No tool points specified");
    }
    for (int p = 0; p < points.size(); p++) {
      Json::Value point = points[p];
      if (!(point.isArray() && point.size() == 2)) {
        throw TrajectoryParsingException("Point in tool path must be an array of two floats");
      }
      float x = point[0].asFloat();
      float y = point[1].asFloat();
      Eigen::Vector2d v(x, y);
      for (size_t i = 0; i < 2; i++) {
        if (v(i) < 0 || v(i) > 1) {
          throw TrajectoryParsingException("Each tool path coordinate must be on the interval [0, 1]");
        }
      }
      result.points.push_back(v);
    }
    return result;
  }

  void parseFile(const std::string &file_name) {
    std::ifstream infile(file_name);
    Json::Value root;
    Json::Reader json_reader;
    if(!json_reader.parse(infile, root)) {
      throw TrajectoryParsingException("Failed to open trajectory JSON file.");
    }
    if (!root) {
      throw TrajectoryParsingException("Failed to parse JSON file.");
    }

    std::string version = root.get("version", "not specified").asString();
    std::cout << "Trajectory version " << version << std::endl;

    const Json::Value sequence_node = root["sequence"];
    if (!sequence_node) {
      throw TrajectoryParsingException("No sequence information found.");
    }

    // Parse the sequence of tool paths
    for (int i = 0; i < sequence_node.size(); i++) {
      sequence.push_back(parseToolPath(sequence_node[i]));
    }
  }
};


#endif // TRAJECTORY_H
