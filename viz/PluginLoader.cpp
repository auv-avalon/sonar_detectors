#include <vizkit/Vizkit3DPlugin.hpp>
#include "AUVAvalonVisualization.hpp"
#include "SonarBeamVisualization.hpp"
#include "SonarFeatureVisualization.hpp"
#include "SonarDepthMapVisualization.hpp"
#include "WallVisualization.hpp"

namespace eslam {
    class QtPluginVizkit : public vizkit::VizkitPluginFactory {
    private:
    public:
	
	QtPluginVizkit() {
	}
	
	/**
	* Returns a list of all available visualization plugins.
	* @return list of plugin names
	*/
        virtual QStringList* getAvailablePlugins() const
	{
	    QStringList *pluginNames = new QStringList();
	    pluginNames->push_back("AUVAvalonVisualization");
	    pluginNames->push_back("SonarBeamVisualization");
            pluginNames->push_back("SonarFeatureVisualization");
	    pluginNames->push_back("SonarDepthMapVisualization");
            pluginNames->push_back("WallVisualization");

	    return pluginNames;
	}
	
        virtual QObject* createPlugin(QString const& pluginName)
        {
	    vizkit::VizPluginBase* plugin = 0;
	    if (pluginName == "AUVAvalonVisualization")
	    {
		plugin = new vizkit::AUVAvalonVisualization();
	    }
	    else if (pluginName == "SonarBeamVisualization")
	    {
		plugin = new vizkit::SonarBeamVisualization();
	    }
	    else if (pluginName == "SonarFeatureVisualization")
            {
                plugin = new vizkit::SonarFeatureVisualization();
            }
            else if (pluginName == "SonarDepthMapVisualization")
            {
                plugin = new vizkit::SonarDepthMapVisualization();
            }
             else if (pluginName == "WallVisualization")
            {
                plugin = new vizkit::WallVisualization();
            }
	      
	    if (plugin) 
	    {
		return plugin;
	    }
	    return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkit, QtPluginVizkit)
}
