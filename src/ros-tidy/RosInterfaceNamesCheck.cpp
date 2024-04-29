#include <string>
#include <utility>

#include "ros-tidy/RosInterfaceNamesCheck.h"

#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "clang/ASTMatchers/ASTMatchers.h"

using namespace clang::ast_matchers;

namespace clang::tidy::ros {

auto matchInterface(const std::string &MainType) {

  auto MatchType =
      anyOf(hasType(hasCanonicalType(
                hasDeclaration(recordDecl(classTemplateSpecializationDecl(
                    allOf(matchesName("std::shared_ptr"),
                          classTemplateSpecializationDecl(hasTemplateArgument(
                              0, refersToType(recordType(hasDeclaration(
                                     recordDecl(hasName(MainType))))))))))))),
            hasType(recordDecl(matchesName(MainType))));

  return std::make_pair(memberExpr(MatchType).bind(MainType),
                        fieldDecl(MatchType).bind(MainType));
}

RosInterfaceNamesCheck::RosInterfaceNamesCheck(StringRef Name,
                                               ClangTidyContext *Context)
    : ClangTidyCheck(Name, Context),
      SubscriptionPrefix(Options.get("SubscriptionPrefix", "_sub")),
      // SubscriptionSuffix(Options.get("SubscriptionSuffix", "")),
      PublisherPrefix(Options.get("PublisherPrefix", "_pub")),
      // PublisherSuffix(Options.get("PublisherSuffix", "")),
      ClientPrefix(Options.get("ClientPrefix", "_client")),
      // ClientSuffix(Options.get("ClientSuffix", "")),
      ServerPrefix(Options.get("ServerPrefix", "_server"))
// ServerSuffix(Options.get("ServerSuffix", ""))
{}

void RosInterfaceNamesCheck::storeOptions(ClangTidyOptions::OptionMap &Opts) {
  Options.store(Opts, "SubscriptionPrefix", SubscriptionPrefix);
  // Options.store(Opts, "SubscriptionSuffix", SubscriptionSuffix);
  Options.store(Opts, "PublisherPrefix", PublisherPrefix);
  // Options.store(Opts, "PublisherSuffix", PublisherSuffix);
  Options.store(Opts, "ClientPrefix", ClientPrefix);
  // Options.store(Opts, "ClientSuffix", ClientSuffix);
  Options.store(Opts, "ServerPrefix", ServerPrefix);
  // Options.store(Opts, "ServerSuffix", ServerSuffix);
}

void RosInterfaceNamesCheck::registerMatchers(MatchFinder *Finder) {
  auto MatchersSub = matchInterface("rclcpp::Subscription");
  Finder->addMatcher(MatchersSub.first, this);
  Finder->addMatcher(MatchersSub.second, this);
  auto MatchersPub = matchInterface("rclcpp::Publisher");
  Finder->addMatcher(MatchersPub.first, this);
  Finder->addMatcher(MatchersPub.second, this);
  auto MatchersClient = matchInterface("rclcpp::Client");
  Finder->addMatcher(MatchersClient.first, this);
  Finder->addMatcher(MatchersClient.second, this);
  auto MatchersService = matchInterface("rclcpp::Service");
  Finder->addMatcher(MatchersService.first, this);
  Finder->addMatcher(MatchersService.second, this);
}

void RosInterfaceNamesCheck::checkInterface(
    const MatchFinder::MatchResult &Result, const std::string &Type,
    const std::string &Prefix, const std::string &Suffix) {
  if (const auto *MatchedExpr = Result.Nodes.getNodeAs<MemberExpr>(Type)) {
    if (!MatchedExpr->getFoundDecl()->getName().startswith(Prefix))
      diag(MatchedExpr->getBeginLoc(),
           "%2 expression %0 need to start with \"%1\"")
          << MatchedExpr->getFoundDecl()->getName() << Prefix << Type
          << FixItHint::CreateInsertion(MatchedExpr->getBeginLoc(), Prefix);
  } else if (const auto *MatchedDecl =
                 Result.Nodes.getNodeAs<FieldDecl>(Type)) {
    if (!MatchedDecl->getIdentifier())
      return;

    if (!MatchedDecl->getName().startswith(Prefix))
      diag(MatchedDecl->getLocation(),
           "%2 expression %0 need to start with \"%1\"")
          << MatchedDecl << Prefix << Type
          << FixItHint::CreateInsertion(MatchedDecl->getLocation(), Prefix);
  }
}

void RosInterfaceNamesCheck::check(const MatchFinder::MatchResult &Result) {
  checkInterface(Result, "rclcpp::Subscription", SubscriptionPrefix,
                 SubscriptionSuffix);
  checkInterface(Result, "rclcpp::Publisher", PublisherPrefix, PublisherSuffix);
  checkInterface(Result, "rclcpp::Client", ClientPrefix, ClientSuffix);
  checkInterface(Result, "rclcpp::Service", ServerPrefix, ServerSuffix);
}

} // namespace clang::tidy::ros
