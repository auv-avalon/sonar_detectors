require 'orocos'
require 'vizkit'

include Orocos
#Orocos.CORBA.name_service = cfg["nameserver"].to_s

#Orocos::CORBA.name_service.ip = "192.168.128.51"
Orocos.initialize

#view3d = Vizkit.default_loader.create_plugin 'vizkit3d::Vizkit3DWidget'
view3d = Vizkit.vizkit3d_widget
view3d.show

#rbs = view3d.createPlugin("base", "RigidBodyStateVisualization")
#laser_scan  = view3d.createPlugin("base", "LaserScanVisualization")
#ps = view3d.createPlugin("uw_localization", "ParticleSetVisualization")
#env = view3d.createPlugin("uw_localization", "MapVisualization")
#sonar = view3d.createPlugin("uw_localization", "SonarPointVisualization")
sonar_beam = view3d.createPlugin("sonar", "SonarBeamVisualization")
view3d.grid.setPluginEnabled(false)

log = Orocos::Log::Replay.open(ARGV)

    log.sonar.sonar_beam.connect_to do |sample,_|
	sonar_beam.updateSonarBeam(sample)
        sample
    end
    
    log.depth_orientation_fusion.pose_samples.connect_to do |sample,_|
        sonar_beam.updateOrientation(sample)
        sample
    end

    Vizkit.control log
    Vizkit.exec

