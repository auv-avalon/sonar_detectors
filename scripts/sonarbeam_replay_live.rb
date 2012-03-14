require 'orocos'
require 'vizkit'
include Orocos

Orocos::CORBA.name_service = "192.168.128.51"
Orocos.initialize

view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
view3d.show
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")

Vizkit.connect_port_to 'sonar', 'BaseScan', :pull => false, :update_frequency => 33 do |sample, _|
    sonarbeamviz.updateSonarBeam(sample)
end

Vizkit.connect_port_to 'orientation_estimator', 'orientation_samples', :pull => false, :update_frequency => 33 do |sample, _|
    sonarbeamviz.updateBodyState(sample)
end

Vizkit.exec
