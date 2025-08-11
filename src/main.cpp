#include <fstream>
#include <iostream>
#include <regex>

#include "../include/ofApp.hpp"
#include "ofMain.h"

const std::regex r_pos =
    std::regex(R"(\((\d+),(\d+),?([XY]{1}_[A-Z]{4,5})?\),)");
const std::regex r_agent = std::regex(R"(^agent(\d+):)");
const std::regex r_history_size = std::regex(R"(history_size=(\d+))");
const std::regex r_step = std::regex(R"(step(\d+):)");

int main(int argc, char *argv[])
{
  // simple arguments check
  if (argc < 3 || !std::ifstream(argv[1]) || !std::ifstream(argv[2])) {
    std::cout << "Please check the arguments, e.g.,\n"
              << "> mapf-visualizer assets/random-32-32-20.map "
                 "assets/demo_random-32-32-20.txt [--lifelong]"
              << std::endl;
    return 0;
  }

  // check for --lifelong option
  bool lifelong_mode = false;
  for (int i = 3; i < argc; i++) {
    if (std::string(argv[i]) == "--lifelong") {
      lifelong_mode = true;
      break;
    }
  }

  // load graph
  Graph G(argv[1]);

  // load plan
  auto solution_file = std::ifstream(argv[2]);
  Solution solution;
  ReferencePath reference_path;
  DynamicGoals dynamic_goals;
  std::string line;
  std::smatch m, results;
  bool in_reference_section = false;
  bool in_goals_section = false;
  int current_agent = -1;
  int current_step = -1;

  std::cout << "Reading solution file..." << std::endl;

  while (getline(solution_file, line)) {
    // 行末の空白を削除
    line = std::regex_replace(line, std::regex(R"(\s+$)"), "");
    
    // goals section detection for lifelong mode
    if (lifelong_mode && line.find("goals=") != std::string::npos) {
      std::cout << "Found goals section for lifelong mode" << std::endl;
      in_goals_section = true;
      continue;
    }
    
    // solution section detection 
    if (line.find("solution=") != std::string::npos) {
      in_goals_section = false;  // exit goals section
      continue;
    }
    
    // parse dynamic goals for lifelong mode
    if (lifelong_mode && in_goals_section && !in_reference_section && (line.find(":(") != std::string::npos)) {
      // Parse agent goals line format: "0:(21,7),(21,7),(21,7),(21,7),(21,7)"
      std::smatch agent_match;
      std::regex r_agent_goals = std::regex(R"(^(\d+):)");
      if (std::regex_search(line, agent_match, r_agent_goals)) {
        int agent_id = std::stoi(agent_match[1].str());
        
        // Extract coordinates for this agent across timesteps
        auto iter = line.cbegin();
        std::vector<Pose> agent_goals_timeline;
        while (iter < line.cend()) {
          auto search_end = std::min(iter + 128, line.cend());
          if (std::regex_search(iter, search_end, m, r_pos)) {
            auto x = std::stoi(m[1].str());
            auto y = std::stoi(m[2].str());
            Orientation o = Orientation::NONE;
            if (m[3].matched) {
              o = Orientation::from_string(m[3].str());
            }
            agent_goals_timeline.push_back(Pose(G.U[G.width * y + x], o));
            iter += m[0].length();
          } else {
            break;
          }
        }
        
        // Store agent goals as timestep goals (same format as solution)
        dynamic_goals.timestep_goals.push_back(agent_goals_timeline);
      }
      continue;
    }

    // 参照軌道セクションの開始を検出
    if (line.find("local_guidance=") != std::string::npos) {
      std::cout << "Found local_guidance section" << std::endl;
      in_reference_section = true;
      continue;
    }

    // 履歴サイズの読み込み
    if (in_reference_section && std::regex_match(line, results, r_history_size)) {
      reference_path.history_size = std::stoi(results[1].str());
      std::cout << "History size: " << reference_path.history_size << std::endl;
      continue;
    }

    // step行の検出
    // if (in_reference_section) {
    //   std::cout << "Checking line for step: '" << line << "'" << std::endl;
    // }
    if (in_reference_section && std::regex_match(line, results, r_step)) {
      // std::cout << "Found step " << results[1].str() << std::endl;
      current_agent = -1;  // 新しいステップの開始時にエージェントをリセット
      current_step = std::stoi(results[1].str());
      continue;
    }

    // エージェント行の検出
    if (in_reference_section) {
      // std::cout << "Checking line for agent: '" << line << "'" << std::endl;
      if (std::regex_search(line, results, r_agent)) {
        current_agent = std::stoi(results[1].str());
        // std::cout << "Found step:" << current_step << " agent:" << current_agent << std::endl;
        if (current_agent >= reference_path.paths.size()) {
          reference_path.paths.resize(current_agent + 1);
          // std::cout << "Resized paths to " << reference_path.paths.size() << std::endl;
        }

        // エージェント行の座標を読み込む
        Config c;
        auto iter = line.cbegin();
        while (iter < line.cend()) {
          auto search_end = std::min(iter + 128, line.cend());
          if (std::regex_search(iter, search_end, m, r_pos)) {
            auto x = std::stoi(m[1].str());
            auto y = std::stoi(m[2].str());
            Orientation o = Orientation::NONE;
            if (m[3].matched) {
              o = Orientation::from_string(m[3].str());
            }
            c.push_back(Pose(G.U[G.width * y + x], o));
            iter += m[0].length();
          } else {
            // 現在の行で座標が見つからない場合、次の行を読み込む
            // if (getline(solution_file, line)) {
            //   line = std::regex_replace(line, std::regex(R"(\s+$)"), "");
            //   std::cout << ": '" << line << "'" << std::endl;
            //   iter = line.cbegin();
            //   continue;
            // }
            break;
          }
        }
        if (!c.empty()) {
          reference_path.paths[current_agent].push_back(c);
          // std::cout << "Added config with " << c.size() << " poses to agent " 
          //           << current_agent << ", total paths: " 
          //           << reference_path.paths[current_agent].size() << std::endl;
        }
        continue;
      }
    }

    // 通常の解の読み込み
    if (!in_reference_section && !in_goals_section && line.find(":(") != std::string::npos) {
      auto iter = line.cbegin();
      Config c;
      
      // Debug: Check if this is agent 0's trajectory
      bool is_agent0 = line.find("0:(") == 0;
      if (is_agent0) {
        std::cout << "DEBUG: Parsing agent 0 solution: " << line.substr(0, 50) << "..." << std::endl;
      }
      
      while (iter < line.cend()) {
        auto search_end = std::min(iter + 128, line.cend());
        if (std::regex_search(iter, search_end, m, r_pos)) {
          auto x = std::stoi(m[1].str());
          auto y = std::stoi(m[2].str());
          Orientation o = Orientation::NONE;
          if (m[3].matched) {
            o = Orientation::from_string(m[3].str());
          }
          c.push_back(Pose(G.U[G.width * y + x], o));
          
          // Debug: Log first 5 positions for agent 0
          if (is_agent0 && c.size() <= 5) {
            std::cout << "DEBUG: Agent 0 position " << c.size() << ": (" << x << "," << y << ")" << std::endl;
          }
          
          iter += m[0].length();
        } else {
          break;
        }
      }
      solution.push_back(c);
      
      if (is_agent0) {
        std::cout << "DEBUG: Agent 0 solution loaded with " << c.size() << " positions" << std::endl;
      }
    }
  }
  solution_file.close();


  // Debug output for dynamic goals
  if (lifelong_mode && !dynamic_goals.timestep_goals.empty()) {
    std::cout << "Loaded dynamic goals with " << dynamic_goals.timestep_goals.size() << " timesteps" << std::endl;
    if (!dynamic_goals.timestep_goals.empty()) {
      std::cout << "First timestep has " << dynamic_goals.timestep_goals[0].size() << " agents" << std::endl;
    }
  }

  // std::cout << "Finished reading. Reference paths size: " << reference_path.paths.size() << std::endl;
  // for (size_t i = 0; i < reference_path.paths.size(); ++i) {
  //   std::cout << "Agent " << i << " has " << reference_path.paths[i].size() << " configs" << std::endl;
  // }

  // visualize
  ofSetupOpenGL(100, 100, OF_WINDOW);
  bool capture_only = false;
  for (int i = 3; i < argc; i++) {
    if (std::string(argv[i]) == "--capture-only") {
      capture_only = true;
      break;
    }
  }
  ofRunApp(new ofApp(&G, &solution, &reference_path, 
                     lifelong_mode ? &dynamic_goals : nullptr,
                     capture_only, lifelong_mode));
  return 0;
}
