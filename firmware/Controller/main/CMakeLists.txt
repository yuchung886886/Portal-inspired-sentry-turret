# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

set(COMPONENT_SRCS
	main.c
	remote_ctrl_task.c
	lcd_task.c
	touch_utils.c
	user_interface_utils.c
	imgs.c)
  
set(COMPONENT_INCLUDE_DIRS
	.)  
	
register_component()
