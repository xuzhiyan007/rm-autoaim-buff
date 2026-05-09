# game status
# 0:未开始比赛 1:准备阶段 2:自检 3:倒计时5s 4：对战中 5:比赛结算中 
# outpost_status
# 0:存活  1：死亡
# enemy_color RED=0  BLUE=1
uint8 PRE_MATCH = 0
uint8 SETUP = 1
uint8 INIT = 2
uint8 FIVE_SEC_CD = 3
uint8 ROUND = 4
uint8 CALCULATION = 5
uint8 game_status
bool outpost_status
bool enemy_color