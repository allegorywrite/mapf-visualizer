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
                 "assets/demo_random-32-32-20.txt"
              << std::endl;
    return 0;
  }

  // load graph
  Graph G(argv[1]);

  // load plan
  auto solution_file = std::ifstream(argv[2]);
  Solution solution;
  ReferencePath reference_path;
  std::string line;
  std::smatch m, results;
  bool in_reference_section = false;
  int current_agent = -1;
  int current_step = -1;

  std::cout << "Reading solution file..." << std::endl;

  while (getline(solution_file, line)) {
    // 行末の空白を削除
    line = std::regex_replace(line, std::regex(R"(\s+$)"), "");

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
    if (!in_reference_section && line.find(":(") != std::string::npos) {
      auto iter = line.cbegin();
      Config c;
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
          break;
        }
      }
      solution.push_back(c);
    }
  }
  solution_file.close();

  // std::cout << "Finished reading. Reference paths size: " << reference_path.paths.size() << std::endl;
  // for (size_t i = 0; i < reference_path.paths.size(); ++i) {
  //   std::cout << "Agent " << i << " has " << reference_path.paths[i].size() << " configs" << std::endl;
  // }

  // visualize
  ofSetupOpenGL(100, 100, OF_WINDOW);
  ofRunApp(new ofApp(&G, &solution, &reference_path,
                     (argc > 3 && std::string(argv[3]) == "--capture-only")));
  return 0;
}
