# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

def configure(conf):
    conf.env['gmp'] = conf.check(mandatory=True, lib='CGAL', uselib_store='CGAL')
    conf.env['mpfr'] = conf.check(mandatory=True, lib='mpfr', uselib_store='mpfr')
    conf.env['boost_system'] = conf.check(mandatory=True, lib='boost_system', uselib_store='boost_system')
    conf.env['boost_thread'] = conf.check(mandatory=True, lib='CGAL', uselib_store='CGAL')
    conf.env['cgal'] = conf.check(mandatory=True, lib='boost_thread', uselib_store='boost_thread')
     


def build(bld):
    module = bld.create_ns3_module('udc-allocator', ['core','mobility'])
    module.source = [
        'model/udc-allocator.cc',
        'helper/udc-allocator-helper.cc',
        ]
    # include CGAL and dependencies... gmp, mpfr, boost_system, boost_thread
    module.use.append('gmp')
    module.use.append('mpfr')
    module.use.append('boost_system')
    module.use.append('boost_thread')
    module.use.append('CGAL')

    module_test = bld.create_ns3_module_test_library('udc-allocator')
    module_test.source = [
        'test/udc-allocator-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'udc-allocator'
    headers.source = [
        'model/udc-allocator.h',
        'helper/udc-allocator-helper.h',
        ]
      

    if bld.env.ENABLE_EXAMPLES:
        bld.recurse('examples')

    # bld.ns3_python_bindings()

