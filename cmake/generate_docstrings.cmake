function(generate_docstrings target python_src library include)
  add_custom_target(
      docstrings ALL
      COMMAND 
         ${Python3_EXECUTABLE} 
          ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/generate_docstrings.py 
          ${python_src} ${library} ${include} ${ARGN} 
      COMMENT "Generate docstrings" 
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/docstrings.h
  )
endfunction()