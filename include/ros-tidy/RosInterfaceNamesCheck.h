#ifndef ROSINTERFACENAMESCHECK_H
#define ROSINTERFACENAMESCHECK_H

#include "clang-tidy/ClangTidyCheck.h"

namespace clang::tidy::ros {

class RosInterfaceNamesCheck : public ClangTidyCheck {
public:
  RosInterfaceNamesCheck(StringRef Name, ClangTidyContext *Context)
      : ClangTidyCheck(Name, Context) {}
  void registerMatchers(ast_matchers::MatchFinder *Finder) override;
  void checkInterface(const ast_matchers::MatchFinder::MatchResult &Result,
                      const std::string &Type, const std::string &Prefix);
  void check(const ast_matchers::MatchFinder::MatchResult &Result) override;
};

} // namespace clang::tidy::ros

#endif // ROSINTERFACENAMESCHECK_H
