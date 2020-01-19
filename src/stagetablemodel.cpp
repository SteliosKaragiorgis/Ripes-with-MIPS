#include "stagetablemodel.h"

#include "parser.h"

#include <vector>

namespace Ripes {

static inline uint32_t indexToAddress(unsigned index) {
    if (ProcessorHandler::get()->getProgram()) {
        return (index * ProcessorHandler::get()->currentISA()->bytes()) +
               ProcessorHandler::get()->getProgram()->getSection(TEXT_SECTION_NAME)->address;
    }
    return 0;
}

StageTableModel::StageTableModel(QObject* parent) : QAbstractTableModel(parent) {}

QVariant StageTableModel::headerData(int section, Qt::Orientation orientation, int role) const {
    if (role != Qt::DisplayRole)
        return QVariant();
    if (orientation == Qt::Horizontal) {
        // Cycle number
        return QString::number(section);
    } else {
        const auto addr = indexToAddress(section);
        return ProcessorHandler::get()->parseInstrAt(addr);
    }
}

int StageTableModel::rowCount(const QModelIndex&) const {
    return ProcessorHandler::get()->getCurrentProgramSize() / ProcessorHandler::get()->currentISA()->bytes();
}

int StageTableModel::columnCount(const QModelIndex&) const {
    return m_cycleStageInfos.size();
}

void StageTableModel::processorWasClocked() {
    gatherStageInfo();
}

void StageTableModel::reset() {
    beginResetModel();
    m_cycleStageInfos.clear();
    endResetModel();
}

void StageTableModel::gatherStageInfo() {
    for (int i = 0; i < ProcessorHandler::get()->getProcessor()->stageCount(); i++) {
        const auto& stageName = ProcessorHandler::get()->getProcessor()->stageName(i);
        m_cycleStageInfos[ProcessorHandler::get()->getProcessor()->getCycleCount()][stageName] =
            ProcessorHandler::get()->getProcessor()->stageInfo(i);
    }
}

QVariant StageTableModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid())
        return QVariant();

    if (role == Qt::TextAlignmentRole) {
        return Qt::AlignCenter;
    }

    if (role != Qt::DisplayRole)
        return QVariant();

    if (!m_cycleStageInfos.count(index.column()))
        return QVariant();

    const uint32_t addr = indexToAddress(index.row());
    const auto& stageInfo = m_cycleStageInfos.at(index.column());
    for (const auto& si : stageInfo) {
        if (si.second.pc == addr && si.second.pc_valid) {
            return si.first;
        }
    }

    return QVariant();
}
}  // namespace Ripes
