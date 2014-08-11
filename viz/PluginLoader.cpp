#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "AUVAvalonVisualization.hpp"
#include "SonarBeamVisualization.hpp"
#include "SonarFeatureVisualization.hpp"
#include "SonarObstaclesVisualization.hpp"
#include "SonarDepthMapVisualization.hpp"
#include "WallVisualization.hpp"

namespace eslam {
    class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
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
            pluginNames->push_back("AvalonSonarBeamVisualization");
            pluginNames->push_back("SonarFeatureVisualization");
            pluginNames->push_back("SonarObstaclesVisualization");
            pluginNames->push_back("SonarDepthMapVisualization");
            pluginNames->push_back("WallVisualization");

	    return pluginNames;
	}
	
        virtual QObject* createPlugin(QString const& pluginName)
        {
	    vizkit3d::VizPluginBase* plugin = 0;
	    if (pluginName == "AUVAvalonVisualization")
	    {
		plugin = new vizkit3d::AUVAvalonVisualization();
	    }
	    else if (pluginName == "AvalonSonarBeamVisualization")
	    {
		plugin = new vizkit3d::AvalonSonarBeamVisualization();
	    }
	    else if (pluginName == "SonarFeatureVisualization")
            {
                plugin = new vizkit3d::SonarFeatureVisualization();
            }
            else if (pluginName == "SonarDepthMapVisualization")
            {
                plugin = new vizkit3d::SonarDepthMapVisualization();
            }
             else if (pluginName == "WallVisualization")
            {
                plugin = new vizkit3d::WallVisualization();
            }
            else if (pluginName == "SonarObstaclesVisualization")
            {
              plugin = new vizkit3d::SonarObstaclesVisualization();
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
