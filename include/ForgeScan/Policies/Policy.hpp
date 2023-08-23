#ifndef FORGE_SCAN_POLICIES_POLICY_HPP
#define FORGE_SCAN_POLICIES_POLICY_HPP

#include <memory>

#include <highfive/H5Easy.hpp>

#include "ForgeScan/Data/Reconstruction.hpp"


namespace forge_scan {

    // Forward definition to allow friend access.
    class Manager;

} // forge_scan


namespace forge_scan {
namespace policies {


/// @brief Base implementation for a Policy that suggests new views for Reconstruction.
class Policy
{
    // ***************************************************************************************** //
    // *                                        FRIENDS                                        * //
    // ***************************************************************************************** //

    /// @brief Requires access to save data.
    friend class forge_scan::Manager;


public:
    // ***************************************************************************************** //
    // *                                 PUBLIC CLASS METHODS                                  * //
    // ***************************************************************************************** //


    /// @brief Queries the Policy to get its suggested view.
    /// @return Gets a constant reference to the best view suggested by the Policy.
    const Extrinsic& getView()
    {
        if (views.empty())
        {
            this->generate();
        }
        assert(views.empty() == false && 
               "The current Policy was unable to suggest views, even after calling generate.");
        return views.front();
    }


    /// @return Number of views a consumer has accepted from this policy.
    size_t numAccepted() const
    {
        return this->accepted_views.size();
    }


    /// @return Number of views a consumer has rejected from this policy.
    size_t numRejected() const
    {
        return this->rejected_views.size();
    }



    // ***************************************************************************************** //
    // *                           PURE VIRTUAL PUBLIC CLASS METHODS                           * //
    // ***************************************************************************************** //


    /// @brief Returns the name of the Policy as a string.
    virtual const std::string& getTypeName() const = 0;


    /// @brief Returns true if the Policy believed the scan is complete.
    virtual bool isComplete() const = 0;


    /// @brief Remove previously generated list of suggested views and generates a new batch.
    virtual void generate() = 0;



protected:
    /* ***************************************************************************************** */
    /*                                 PROTECTED CLASS METHODS                                   */
    /* ***************************************************************************************** */


    /// @brief Protected constructor for derived classes only.
    /// @param reconstruction Shared pointer to the Reconstruction that the Policy suggests
    ///                       new views for.
    Policy(const std::shared_ptr<const data::Reconstruction>& reconstruction)
        : reconstruction(reconstruction)
    {

    }


    /// @brief Accepts the view returned by this->getView and removed it from the list.
    /// @param count Total number of views accepted/rejected from all policies.
    ///              This is just used as a simple unique, ordered identifier.
    /// @throws std::runtime_error if the Policy has no suggested views.
    void acceptView(const size_t& count = 0)
    {
        if ( !this->views.empty() )
        {
            this->accepted_views.push_back( {count, this->views.front()} );
            return views.pop_front();
        }
        throw std::runtime_error("The Policy has no proposed views. Cannot accept a view.");
    }


    /// @brief Rejects the view returned by this->getView and removed it from the list.
    /// @param count Total number of views accepted/rejected from all policies.
    ///              This is just used as a simple unique, ordered identifier.
    /// @throws std::runtime_error if the Policy has no suggested views.
    void rejectView(const size_t& count = 0)
    {
        if ( !this->views.empty() )
        {
            this->rejected_views.push_back( {count, this->views.front()} );
            this->views.pop_front();
        }
        throw std::runtime_error("The Policy has no proposed views. Cannot reject a view.");
    }


    /// @brief Saves the rejected views, and their order identifier, to the HDF5 file.
    /// @param file File to write to.
    /// @param policy_name Name of the derived Policy class.
    void saveRejectedViews(H5Easy::File& file, const std::string& policy_name) const
    {
        const std::string hdf5_data_root = "/" FS_HDF5_POLICY_GROUP "/" + policy_name + "/REJECT";
        std::stringstream ss;
        for (const auto& reject : this->rejected_views)
        {
            ss << hdf5_data_root << "/" << reject.first;
            H5Easy::dump(file, ss.str(), reject.second.matrix());
            ss.str(std::string());
        }
    }



    // ***************************************************************************************** //
    // *                              PROTECTED VIRTUAL METHODS                                * //
    // ***************************************************************************************** //


    /// @brief Runs when a Policy is added to the Manager.
    ///        This give the Policy the option to add data channels it might need.
    virtual void setup()
    {

    }


    /// @brief Save the Policy information into an HDF5 file.
    /// @param file The HDF5 file to write to.
    /// @param g_policy The specific group to write Policy information in.
    virtual void save(H5Easy::File& file, HighFive::Group& g_policy) const = 0;



    // ***************************************************************************************** //
    // *                               PROTECTED STATIC METHODS                                * //
    // ***************************************************************************************** //


    /// @param policy_name Name of the policy.
    /// @return Path in an HDF5 file  where the Metric should read/write information from.
    static std::string getDatasetPathHDF5(const std::string& metric_name)
    {
        return "/" FS_HDF5_METRIC_GROUP "/" + metric_name + "/data";
    }



    // ***************************************************************************************** //
    // *                               PROTECTED CLASS MEMBERS                                 * //
    // ***************************************************************************************** //


    /// @brief Storage for the suggested views the Policy has generated.
    std::list<Extrinsic> views;

    /// @brief Storage for the views that were a Policy and accepted by the consumer.
    std::list<std::pair<size_t, Extrinsic>> accepted_views;


    /// @brief Storage for the views that were generated by the Policy but rejected by the consumer.
    std::list<std::pair<size_t, Extrinsic>> rejected_views;

    /// @brief Reference to the Reconstruction class. Some Policies use this to add a specific data
    ///        channel which they require.
    std::shared_ptr<const data::Reconstruction> reconstruction;
};


} // namespace policies
} // namespace forge_scan


#endif // FORGE_SCAN_POLICIES_POLICY_HPP