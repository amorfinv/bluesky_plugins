''' Traffic OpenGL visualisation for nicer drone routes. '''
from bluesky.ui.qtgl import glhelpers as glh
from bluesky.ui.qtgl.gltraffic import Traffic
import bluesky as bs
from bluesky import settings
from bluesky.ui import palette

import numpy as np

### Initialization function of plugin.
def init_plugin():
    config = {
        'plugin_name':     'NICETRAFFIC',
        'plugin_type':     'gui',
        }

    return config

# Register settings defaults
settings.set_variable_defaults(
    text_size=13, ac_size=16,
    asas_vmin=200.0, asas_vmax=500.0)

palette.set_default_colours(
    aircraft=(0, 255, 0),
    conflict=(255, 160, 0),
    route=(255, 0, 255),
    trails=(0, 255, 255))

# Static defines
MAX_NAIRCRAFT = 10000
MAX_NCONFLICTS = 25000
MAX_ROUTE_LENGTH = 500
ROUTE_SIZE = 5000
TRAILS_SIZE = 1000000

class NiceTraffic(Traffic):


    def create(self):
        ac_size = settings.ac_size
        wpt_size = settings.wpt_size
        self.hdg.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.lat.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.lon.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.alt.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.tas.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.color.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)
        self.lbl.create(MAX_NAIRCRAFT * 24, glh.GLBuffer.UsagePattern.StreamDraw)
        self.asasn.create(MAX_NAIRCRAFT * 24, glh.GLBuffer.UsagePattern.StreamDraw)
        self.asase.create(MAX_NAIRCRAFT * 24, glh.GLBuffer.UsagePattern.StreamDraw)
        self.rpz.create(MAX_NAIRCRAFT * 4, glh.GLBuffer.UsagePattern.StreamDraw)

        self.ssd.create(lat1=self.lat, lon1=self.lon, alt1=self.alt,
                        tas1=self.tas, trk1=self.hdg)
        self.ssd.set_attribs(selssd=MAX_NAIRCRAFT, instance_divisor=1,
                             datatype=glh.gl.GL_UNSIGNED_BYTE, normalize=True)
        self.ssd.set_attribs(lat0=self.lat, lon0=self.lon,
                             alt0=self.alt, tas0=self.tas,
                             trk0=self.hdg, asasn=self.asasn,
                             asase=self.asase, instance_divisor=1)

        #self.protectedzone.create(radius=1.0)
        self.protectedzone.create(radius=0.5)
        self.protectedzone.set_attribs(lat=self.lat, lon=self.lon, scale=self.rpz,
                                       color=self.color, instance_divisor=1)

        acvertices = np.array([(0.0, 0.5 * ac_size), (-0.5 * ac_size, -0.5 * ac_size),
                               (0.0, -0.25 * ac_size), (0.5 * ac_size, -0.5 * ac_size)],
                              dtype=np.float32)
        self.ac_symbol.create(vertex=acvertices)

        self.ac_symbol.set_attribs(lat=self.lat, lon=self.lon, color=self.color,
                                   orientation=self.hdg, instance_divisor=1)

        self.aclabels.create(self.lbl, self.lat, self.lon, self.color,
                             (ac_size, -0.5 * ac_size), instanced=True)

        self.cpalines.create(vertex=MAX_NCONFLICTS * 16, color=palette.conflict, usage=glh.GLBuffer.UsagePattern.StreamDraw)

        # ------- Aircraft Route -------------------------
        self.route.create(vertex=ROUTE_SIZE * 8, color=palette.route, usage=glh.GLBuffer.UsagePattern.DynamicDraw)

        self.routelbl.create(ROUTE_SIZE * 24, ROUTE_SIZE * 4, ROUTE_SIZE * 4,
                             palette.route, (wpt_size, 0.5 * wpt_size), instanced=True)
        rwptvertices = np.array([(-0.2 * wpt_size, -0.2 * wpt_size),
                                 (0.0,            -0.8 * wpt_size),
                                 (0.2 * wpt_size, -0.2 * wpt_size),
                                 (0.8 * wpt_size,  0.0),
                                 (0.2 * wpt_size,  0.2 * wpt_size),
                                 (0.0,             0.8 * wpt_size),
                                 (-0.2 * wpt_size,  0.2 * wpt_size),
                                 (-0.8 * wpt_size,  0.0)], dtype=np.float32)
        self.rwaypoints.create(vertex=rwptvertices, color=palette.route)
        self.rwaypoints.set_attribs(lat=self.routelbl.lat, lon=self.routelbl.lon, instance_divisor=1)

        # # --------Aircraft Trails------------------------------------------------
        self.traillines.create(vertex=TRAILS_SIZE * 16, color=palette.trails)
        self.initialized = True


    def draw(self):
        ''' Draw all traffic graphics. '''
        # Get data for active node
        actdata = bs.net.get_nodedata()
        if actdata.naircraft == 0 or not actdata.show_traf:
            return

        # Send the (possibly) updated global uniforms to the buffer
        self.shaderset.set_vertex_scale_type(self.shaderset.VERTEX_IS_LATLON)
        self.shaderset.enable_wrap(False)

        self.route.draw()
        self.cpalines.draw()
        self.traillines.draw()

        # --- DRAW THE INSTANCED AIRCRAFT SHAPES ------------------------------
        # update wrap longitude and direction for the instanced objects
        self.shaderset.enable_wrap(True)

        # PZ circles only when they are bigger than the A/C symbols
        if actdata.show_pz and actdata.zoom >= 0.15:
            self.shaderset.set_vertex_scale_type(
                self.shaderset.VERTEX_IS_METERS)
            self.protectedzone.draw(n_instances=actdata.naircraft)

        self.shaderset.set_vertex_scale_type(self.shaderset.VERTEX_IS_SCREEN)

        # Draw traffic symbols
        self.ac_symbol.draw(n_instances=actdata.naircraft)

        # remove route labels
        # if self.routelbl.n_instances:
        #     self.rwaypoints.draw(n_instances=self.routelbl.n_instances)
        #     # self.routelbl.draw()

        if actdata.show_lbl:
            self.aclabels.draw(n_instances=actdata.naircraft)

        # SSD
        if actdata.ssd_all or actdata.ssd_conflicts or len(actdata.ssd_ownship) > 0:
            ssd_shader = glh.ShaderSet.get_shader('ssd')
            ssd_shader.bind()
            glh.gl.glUniform3f(ssd_shader.uniforms['Vlimits'].loc, self.asas_vmin **
                           2, self.asas_vmax ** 2, self.asas_vmax)
            glh.gl.glUniform1i(ssd_shader.uniforms['n_ac'].loc, actdata.naircraft)
            self.ssd.draw(vertex_count=actdata.naircraft,
                          n_instances=actdata.naircraft)

