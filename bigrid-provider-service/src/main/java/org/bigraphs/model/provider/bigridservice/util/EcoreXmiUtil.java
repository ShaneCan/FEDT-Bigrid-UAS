package org.bigraphs.model.provider.bigridservice.util;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.resource.impl.ResourceSetImpl;
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl;

import java.io.ByteArrayOutputStream;
import java.util.Collections;

public class EcoreXmiUtil {

    /**
     * Serialize an EObject into an XMI string.
     */
    public static String toXmiString(EObject eObject) throws Exception {
        // Prepare EMF ResourceSet and register .xmi factory
        ResourceSet rs = new ResourceSetImpl();
        rs.getResourceFactoryRegistry()
          .getExtensionToFactoryMap()
          .put(Resource.Factory.Registry.DEFAULT_EXTENSION, new XMIResourceFactoryImpl());

        // Create a temporary in-memory resource
        Resource res = rs.createResource(URI.createURI("dummy.xmi"));
        res.getContents().add(eObject);

        // Save to a byte array
        try (ByteArrayOutputStream out = new ByteArrayOutputStream()) {
            res.save(out, Collections.emptyMap());
            return out.toString("UTF-8");
        }
    }
}