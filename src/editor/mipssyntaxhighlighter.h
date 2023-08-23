#pragma once

#include <QRegularExpression>
#include <set>

#include "syntaxhighlighter.h"

namespace Ripes {

class MipsSyntaxHighlighter : public SyntaxHighlighter {
public:
  MipsSyntaxHighlighter(QTextDocument *parent,
                      std::shared_ptr<Assembler::Errors> errors,
                      const std::set<QString> &supportedOpcodes);
  void syntaxHighlightBlock(const QString &text) override;

private:
  struct HighlightingRule {
    QRegularExpression pattern;
    QTextCharFormat format;
  };
  QVector<HighlightingRule> m_highlightingRules;

  QTextCharFormat registerFormat;
  QTextCharFormat labelFormat;
  QTextCharFormat directiveFormat;
  QTextCharFormat instructionFormat;
  QTextCharFormat stringFormat;
  QTextCharFormat charFormat;
  QTextCharFormat commentFormat;
  QTextCharFormat immediateFormat;
};

} // namespace Ripes
