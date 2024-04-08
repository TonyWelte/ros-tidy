#include "clang-tidy/ClangTidy.h"
#include "clang-tidy/ClangTidyModule.h"
#include "clang-tidy/ClangTidyModuleRegistry.h"

#include "ros-tidy/RosInterfaceNamesCheck.h"

using namespace clang::ast_matchers;

namespace clang::tidy {
namespace ros {

/// This module is for ROS-specific checks.
class RosTidyModule : public ClangTidyModule {
public:
  void addCheckFactories(ClangTidyCheckFactories &CheckFactories) override {
    CheckFactories.registerCheck<RosInterfaceNamesCheck>("ros-interface-names");
  }
};

// Register the RosTidyTidyModule using this statically initialized variable.
static ClangTidyModuleRegistry::Add<RosTidyModule>
    X("ros-tidy-module", "Adds ROS specific checks.");
} // namespace ros

// This anchor is used to force the linker to link in the generated object file
// and thus register the RosTidyModule.
volatile int RosTidyModuleAnchorSource = 0;

} // namespace clang::tidy
