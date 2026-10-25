std_msgs/Header header
string poi
uint8 status             # 0 NORMAL / 1 ABNORMAL / 2 UNCERTAIN / 3 TIMEOUT / 4 NO_FRAME / 5 NO_MODEL / 6 ERROR
bool simulated           # true = 模拟结果，不得当作真实识别结论
string detail
string detections_json
