require 'orocos'
require 'Qt4'
require 'qwt'
require 'vizkit'
include Orocos
Orocos.initialize

if ARGV[0] == nil
	puts "Please add a path to a logfile, containing sonarscans, as argument!"
	exit
end

curve = Qwt::PlotCurve.new
plot = Qwt::Plot.new
plot.setAxisScale(0,0,200)
plot.setAxisScale(2,0,300)
curve.attach(plot)
plot.show
curve.show

log = Orocos::Log::Replay.open(ARGV)

log.sonar.BaseScan :type => :buffer, :size => 100  do |sample|
    b = Array.new(sample.beam.length)
    a = Array.new(sample.beam.length)
    for i in 0..sample.beam.length-1 do
        a[i] = i
        b[i] = sample.beam[i]
    end
    curve.setData(a,b)
    plot.replot
    sample
end


Vizkit.control log
Vizkit.exec 
