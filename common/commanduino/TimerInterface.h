#pragma once

class TimerInterface {

  /// \brief the time since the start of the program in milliseconds
public:
  virtual unsigned long programTimeMs() = 0;
};

template <typename T>
class Singleton
{
 public:
  static T* Instance() {
    if (m_instance == nullptr) {
      m_instance = new T();
    }
    return m_instance;
  }
 private:
  static T* m_instance;
};
