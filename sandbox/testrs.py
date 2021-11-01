import pyrealsense2 as rs
p = rs.pipeline()
c = rs.config()
pw = rs.pipeline_wrapper(p)
pp = c.resolve(pw)

