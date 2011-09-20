require 'orocos'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

view3d = Vizkit.default_loader.create_widget('vizkit::Vizkit3DWidget')
view3d.show
sonarbeamviz = view3d.createPlugin("sonarbeam","SonarBeamVisualization")

log = Orocos::Log::Replay.open(ARGV)

log.sonar.BaseScan :type => :buffer, :size => 100  do |sample|
    sonarbeamviz.updateSonarScan(sample)
    #puts sample.angle
    sample
end

#log.orientation_estimator.orientation_samples :type => :buffer, :size => 100  do |sample|
#    sonarbeamviz.updateBodyState(sample)
#    sample
#end 

Vizkit.control log
Vizkit.exec 
