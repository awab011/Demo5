uint8 slot_num       # 1-9 (rack) or 1-12 (forest)
uint8 col            # 0=left  1=middle  2=right
uint8 row            # 0=bottom  1=middle  2=top
string kfs_type      # "TEAM_KFS" | "OPP_KFS" | "Fake_KFS" | "R1_KFS" | "R2_KFS"
float32 confidence   # 0.0-1.0
float32 distance_m   # robot-frame distance to centroid
