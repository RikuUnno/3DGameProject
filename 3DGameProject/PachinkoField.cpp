#include "PachinkoField.h"

#include "ObjectFactory.h"
#include "ObjectManager.h"

// PachinkoField の自動登録
namespace {
struct PachinkoFieldAutoRegister {
	PachinkoFieldAutoRegister() {
		ObjectFactory::Instance().RegisterCreator(	// PachinkoField_Front
			PachinkoField_Front::StaticPoolKey(),
			[](const VariantMap&) { return std::make_unique<PachinkoField_Front>(); });
		ObjectFactory::Instance().RegisterCreator(	// PachinkoField_Back
			PachinkoField_Back::StaticPoolKey(),
			[](const VariantMap&) { return std::make_unique<PachinkoField_Back>(); });
		ObjectFactory::Instance().RegisterCreator(	// PachinkoField_Side
			PachinkoField_Side::StaticPoolKey(),
			[](const VariantMap&) { return std::make_unique<PachinkoField_Side>(); });

		ObjectManager::Instance().RegisterPool(PachinkoField_Front::StaticPoolKey(), 8);
		ObjectManager::Instance().RegisterPool(PachinkoField_Back::StaticPoolKey(), 8);
		ObjectManager::Instance().RegisterPool(PachinkoField_Side::StaticPoolKey(), 8);
	}
};

// グローバル変数として PachinkoFieldAutoRegister のインスタンスを作成し、コンストラクタで自動登録を行う
static PachinkoFieldAutoRegister g_pachinkoFieldAutoRegister;
}