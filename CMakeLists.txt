# If we don't need RTTI or EH, there's no reason to export anything
# from the hello plugin.

add_llvm_library(gsohc_opt MODULE BUILDTREE_ONLY
  gsohc_main_analysis.cpp

  DEPENDS
  intrinsics_gen
  PLUGIN_TOOL
  opt
  )
