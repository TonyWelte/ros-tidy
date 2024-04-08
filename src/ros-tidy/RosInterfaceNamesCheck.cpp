#include <string>
#include <utility>

#include "ros-tidy/RosInterfaceNamesCheck.h"

#include "clang/AST/Decl.h"
#include "clang/AST/Expr.h"
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "clang/ASTMatchers/ASTMatchers.h"

using namespace clang::ast_matchers;

namespace clang::tidy::ros {

auto matchInterface(const std::string &MainType, const std::string &BindName) {
  return std::make_pair(
      memberExpr(
          anyOf(hasType(hasCanonicalType(hasDeclaration(
                    recordDecl(classTemplateSpecializationDecl(allOf(
                        matchesName("std::shared_ptr"),
                        classTemplateSpecializationDecl(hasTemplateArgument(
                            0, refersToType(recordType(hasDeclaration(
                                   recordDecl(hasName(MainType))))))))))))),
                hasType(recordDecl(matchesName(MainType)))))
          .bind(BindName),
      fieldDecl(
          anyOf(hasType(hasCanonicalType(hasDeclaration(
                    recordDecl(classTemplateSpecializationDecl(allOf(
                        matchesName("std::shared_ptr"),
                        classTemplateSpecializationDecl(hasTemplateArgument(
                            0, refersToType(recordType(hasDeclaration(
                                   recordDecl(hasName(MainType))))))))))))),
                hasType(recordDecl(matchesName(MainType)))))
          .bind(BindName));
}

void RosInterfaceNamesCheck::registerMatchers(MatchFinder *Finder) {
  auto matchers_sub = matchInterface("rclcpp::Subscription", "sub");
  Finder->addMatcher(matchers_sub.first, this);
  Finder->addMatcher(matchers_sub.second, this);
  auto matchers_pub = matchInterface("rclcpp::Publisher", "pub");
  Finder->addMatcher(matchers_pub.first, this);
  Finder->addMatcher(matchers_pub.second, this);
  auto matchers_client = matchInterface("rclcpp::Client", "client");
  Finder->addMatcher(matchers_client.first, this);
  Finder->addMatcher(matchers_client.second, this);
  auto matchers_service = matchInterface("rclcpp::Service", "service");
  Finder->addMatcher(matchers_service.first, this);
  Finder->addMatcher(matchers_service.second, this);
}

void RosInterfaceNamesCheck::checkInterface(
    const MatchFinder::MatchResult &Result, const std::string &Type,
    const std::string &Prefix) {
  if (const auto *MatchedExpr = Result.Nodes.getNodeAs<MemberExpr>(Prefix)) {
    const auto *MatchedDecl = dyn_cast<FieldDecl>(MatchedExpr->getMemberDecl());
    if (!MatchedDecl->getIdentifier() ||
        MatchedDecl->getName().startswith("_" + Prefix + "_"))
      return;

    diag(MatchedExpr->getExprLoc(),
         "%2 expression %0 need to start with \"_%1\"")
        << MatchedDecl << Prefix << Type
        << FixItHint::CreateInsertion(MatchedExpr->getExprLoc(), "_" + Prefix);
    diag(MatchedExpr->getExprLoc(), std::string() + "insert '_" + Prefix + "'",
         DiagnosticIDs::Note);
  } else if (const auto *MatchedDecl =
                 Result.Nodes.getNodeAs<FieldDecl>(Prefix)) {
    if (!MatchedDecl->getIdentifier() ||
        MatchedDecl->getName().startswith("_" + Prefix + "_"))
      return;

    diag(MatchedDecl->getLocation(),
         "%2 declaration %0 need to start with \"_%1\"")
        << MatchedDecl << Prefix << Type
        << FixItHint::CreateInsertion(MatchedDecl->getLocation(), "_" + Prefix);
    diag(MatchedDecl->getLocation(), std::string() + "insert '_" + Prefix + "'",
         DiagnosticIDs::Note);
  }
}

void RosInterfaceNamesCheck::check(const MatchFinder::MatchResult &Result) {
  checkInterface(Result, "rclcpp::Subscription", "sub");
  checkInterface(Result, "rclcpp::Publisher", "pub");
  checkInterface(Result, "rclcpp::Client", "client");
  checkInterface(Result, "rclcpp::Service", "service");
}

} // namespace clang::tidy::ros
