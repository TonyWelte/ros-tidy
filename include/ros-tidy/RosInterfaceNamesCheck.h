#ifndef ROSINTERFACENAMESCHECK_H
#define ROSINTERFACENAMESCHECK_H

#include "clang-tidy/ClangTidyCheck.h"

namespace clang::tidy::ros {

class RosInterfaceNamesCheck : public ClangTidyCheck {
  const std::string SubscriptionPrefix;
  const std::string SubscriptionSuffix;
  const std::string PublisherPrefix;
  const std::string PublisherSuffix;
  const std::string ClientPrefix;
  const std::string ClientSuffix;
  const std::string ServerPrefix;
  const std::string ServerSuffix;

public:
  RosInterfaceNamesCheck(StringRef Name, ClangTidyContext *Context);
  void storeOptions(ClangTidyOptions::OptionMap &Opts) override;
  void registerMatchers(ast_matchers::MatchFinder *Finder) override;
  void checkInterface(const ast_matchers::MatchFinder::MatchResult &Result,
                      const std::string &Type, const std::string &Prefix,
                      const std::string &Suffix);
  void check(const ast_matchers::MatchFinder::MatchResult &Result) override;
};

} // namespace clang::tidy::ros

#endif // ROSINTERFACENAMESCHECK_H
