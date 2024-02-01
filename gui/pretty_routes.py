''' Traffic OpenGL visualisation for nicer drone routes. '''
from bluesky.ui.qtgl import glhelpers as glh
from bluesky.ui.qtgl.gltraffic import Traffic
import bluesky as bs



### Initialization function of plugin.
def init_plugin():
    config = {
        'plugin_name':     'NICETRAFFIC',
        'plugin_type':     'gui',
        }

    return config


class NiceTraffic(Traffic):

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

