#include "ros-tidy/RosIncludeOrderCheck.h"

#include "clang/Basic/FileEntry.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Lex/PPCallbacks.h"
#include "clang/Lex/Preprocessor.h"
#include "llvm/ADT/STLExtras.h"

#include <iostream>

#include <map>

namespace clang::tidy::ros {

namespace {

enum class IncludeType {
  ROS,
  LOCAL,
  MSG,
  OTHER,
};

class RosIncludeOrderPPCallbacks : public PPCallbacks {
public:
  explicit RosIncludeOrderPPCallbacks(ClangTidyCheck &Check,
                                      const SourceManager &SM)
      : LookForMainModule(true), Check(Check), SM(SM) {}
  void InclusionDirective(SourceLocation HashLoc, const Token &IncludeTok,
                          StringRef FileName, bool IsAngled,
                          CharSourceRange FilenameRange, const FileEntry *File,
                          StringRef SearchPath, StringRef RelativePath,
                          const Module *Imported,
                          SrcMgr::CharacteristicKind FileType) override;
  void EndOfMainFile() override;

private:
  struct IncludeDirective {
    SourceLocation Loc;    ///< '#' location in the include directive
    CharSourceRange Range; ///< SourceRange for the file name
    std::string Filename;  ///< Filename as a string
    std::string SearchPath;
  };

  typedef std::vector<IncludeDirective> FileIncludes;
  std::map<clang::FileID, FileIncludes> IncludeDirectives;
  bool LookForMainModule;

  ClangTidyCheck &Check;
  const SourceManager &SM;

  IncludeType getType(IncludeDirective ID);
};
} // namespace

void RosIncludeOrderCheck::registerPPCallbacks(const SourceManager &SM,
                                               Preprocessor *PP,
                                               Preprocessor *ModuleExpanderPP) {
  PP->addPPCallbacks(::std::make_unique<RosIncludeOrderPPCallbacks>(*this, SM));
}

void RosIncludeOrderPPCallbacks::InclusionDirective(
    SourceLocation HashLoc, const Token &IncludeTok, StringRef FileName,
    bool IsAngled, CharSourceRange FilenameRange, const FileEntry *File,
    StringRef SearchPath, StringRef RelativePath, const Module *Imported,
    SrcMgr::CharacteristicKind FileType) {
  // We recognize the first include as a special main module header and want
  // to leave it in the top position.
  IncludeDirective ID = {HashLoc, FilenameRange, std::string(FileName),
                         std::string(SearchPath)};

  IncludeDirectives[SM.getFileID(HashLoc)].push_back(std::move(ID));
}

IncludeType RosIncludeOrderPPCallbacks::getType(IncludeDirective ID) {
  if (StringRef(ID.Filename).rsplit("/").first.split("/").second == "msg") {
    return IncludeType::MSG;
  }

  if (StringRef(ID.SearchPath).startswith("/opt/ros/")) {
    return IncludeType::ROS;
  }

  return IncludeType::OTHER;
}

void RosIncludeOrderPPCallbacks::EndOfMainFile() {
  LookForMainModule = true;
  if (IncludeDirectives.empty())
    return;

  for (const auto &[FileID, ID] : IncludeDirectives) {
    std::cout << FileID.getHashValue() << std::endl;

    std::vector<std::set<std::string>> SortedIncludes;

    for (const auto &IncDir : ID) {
      // Get type priority
      const auto Type = getType(IncDir);

      std::cout << IncDir.Filename << " in directory: " << IncDir.SearchPath
                << ", type: " << (int)Type << std::endl;
    }

    break; // Ignore other files for development
  }

  std::cout << "End of main file" << std::endl;

  // TODO

  IncludeDirectives.clear();
}

} // namespace clang::tidy::ros
