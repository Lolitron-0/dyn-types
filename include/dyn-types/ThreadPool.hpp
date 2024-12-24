#pragma once
#include <functional>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
public:
  void addTask(const std::function<void(void)>& task) {
  }

private:
  std::queue<std::function<void(void)>> tasks;
  std::vector<std::jthread> threads;
};
