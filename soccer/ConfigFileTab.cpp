#include <ConfigFileTab.hpp>
#include <ConfigFileTab.moc>

ConfigFileTab::ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent)
: QWidget(parent),
  _config(config)
{
	_ui.setupUi(this);
}

void ConfigFileTab::save(QString filename) {

}

void ConfigFileTab::load(QString filename) {

}

void ConfigFileTab::on_loadConfig_clicked() {

}

void ConfigFileTab::on_saveConfig_clicked() {

}




