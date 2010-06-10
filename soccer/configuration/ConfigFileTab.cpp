#include <configuration/ConfigFileTab.hpp>
#include <ConfigFileTab.moc>

#include <boost/foreach.hpp>

ConfigFileTab::ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent)
: QWidget(parent), _model(new ConfigFileModel(config, this))
{
	_ui.setupUi(this);
	_ui.paramTree->setModel(_model);
}

void ConfigFileTab::save(QString filename) {

}

void ConfigFileTab::load(QString filename) {

}

void ConfigFileTab::on_loadConfig_clicked() {
	// TODO: add load code here
}

void ConfigFileTab::on_saveConfig_clicked() {
	// TODO: add save code here
}




