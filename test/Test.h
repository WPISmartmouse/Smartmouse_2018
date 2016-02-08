class TestResult {
  public:
    TestResult();
    TestResult(bool success, int status, const char *msg);
    bool success;
    int status;
    const char *msg;
};
