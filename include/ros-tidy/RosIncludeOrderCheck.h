#ifndef ROSINCLUDEORDERCHECK_H
#define ROSINCLUDEORDERCHECK_H

#include "clang-tidy/ClangTidyCheck.h"

namespace clang::tidy::ros {

class RosIncludeOrderCheck : public ClangTidyCheck {
public:
  RosIncludeOrderCheck(StringRef Name, ClangTidyContext *Context)
      : ClangTidyCheck(Name, Context) {}
  void registerPPCallbacks(const SourceManager &SM, Preprocessor *PP,
                           Preprocessor *ModuleExpanderPP) override;
};

} // namespace clang::tidy::ros

#endif // ROSINCLUDEORDERCHECK_H
