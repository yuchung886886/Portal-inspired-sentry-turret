# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

set(COMPONENT_SRCS
  main.c
  remote_ctrl_task.c
  camera_utils.c
  orientation_ctrl_task.c
  fire_ctrl_task.c
  speaker_ctrl_task.c
  range_finder_utils.c)
  
set(COMPONENT_INCLUDE_DIRS
	.)  
	
set(COMPONENT_EMBED_FILES
	sound/turret_deploy.pcm 
	sound/turret_alarm.pcm
	sound/turret_hello.pcm 
	sound/turret_anyone_there.pcm 
	sound/turret_malfunction.pcm 
	sound/turret_target_acquired.pcm 
	sound/turret_retract.pcm
	sound/turret_goodnight.pcm
	sound/turret_are_you_still_there.pcm
	sound/turret_opera_singing.pcm)	

register_component()