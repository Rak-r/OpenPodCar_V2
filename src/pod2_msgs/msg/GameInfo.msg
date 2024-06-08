# game played tells if the game has been played or not
# yielding tells if the players are yielding or not
# player1pose & player2pose tells the discreate position of players in game matrix
# player1yieldprob & player2yieldprobprobabilit of yielding


std_msgs/Header header
bool game_played       
bool yielding          
int32 player1discrete    
int32 player2discrete    
float64 player1yieldprob
float64 player2yieldprob
geometry_msgs/Pose player1_pose
geometry_msgs/Pose player2_pose